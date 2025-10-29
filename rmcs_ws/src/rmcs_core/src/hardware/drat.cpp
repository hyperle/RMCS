#include <atomic>
#include <memory>
#include <thread>
#include <deque>

#include <librmcs/client/cboard.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_utility/fps_counter.hpp>
#include <serial/serial.h>

#include "hardware/device/bmi088.hpp"
#include "hardware/device/hipnuc.hpp"
#include "filter/low_pass_filter.hpp"

namespace rmcs_core::hardware {

class Dart 
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    Dart()
        : Node(
              get_component_name(), 
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        
        command_component_ = create_partner_component<DartCommand>(get_component_name() + "_command", *this);
        
        first_sample_time_ = this->get_parameter("first_sample_time").as_double();
        final_sample_time_ = this->get_parameter("final_sample_time").as_double();
        register_output("/tf", tf_);
        
        top_board_ = std::make_unique<TopBoard>(
            *this, *command_component_,
            static_cast<int>(get_parameter("usb_pid_top_board").as_int()));
    }

    ~Dart() override = default;

    void update() override {
        top_board_->update();
    }

    void command_update() {
        top_board_->command_update();
    }

private:
    class DartCommand : public rmcs_executor::Component {
    public:
        explicit DartCommand(Dart& dart)
            : dart_(dart) {}

        void update() override { dart_.command_update(); }

        Dart& dart_;
    };
    std::shared_ptr<DartCommand> command_component_;

    class TopBoard final : private librmcs::client::CBoard {
    public:
        friend class Dart;
        explicit TopBoard(Dart& dart, DartCommand& dart_command, int usb_pid = -1)
            : librmcs::client::CBoard(usb_pid)
            , dart_(dart)
            , tf_(dart.tf_)
            , imu_(1000, 0.2, 0.0)
            , roll_filter_(20.0, 1000.0)
            , pitch_filter_(20.0, 1000.0)
            , yaw_filter_(20.0, 1000.0) {

            dart.register_output("/gimbal/state/roll", gimbal_state_roll_);
            dart.register_output("/gimbal/state/pitch", gimbal_state_pitch_);
            dart.register_output("/gimbal/state/yaw", gimbal_state_yaw_);
            dart.register_output("/gimbal/state/filtered_roll", gimbal_state_filtered_roll_);
            dart.register_output("/gimbal/state/filtered_pitch", gimbal_state_filtered_pitch_);
            dart.register_output("/gimbal/state/filtered_yaw", gimbal_state_filtered_yaw_);
            dart.register_output("/gimbal/state/quaternion_x", quaternion_x_);
            dart.register_output("/gimbal/state/quaternion_y", quaternion_y_);
            dart.register_output("/gimbal/state/quaternion_z", quaternion_z_);
            dart.register_output("/gimbal/state/quaternion_w", quaternion_w_);

            external_imu_thread_ = std::jthread([this, &dart](const std::stop_token& stop_token) {
                external_imu_thread_main(
                    stop_token, dart.get_parameter("external_imu_port").as_string(),
                    dart.get_logger());
            });

            event_thread_ = std::thread([this]() { handle_events(); });
            
            start_time_ = std::chrono::steady_clock::now();
            calibration_complete_ = false;
        }

        ~TopBoard() override {
            stop_handling_events();
            if (event_thread_.joinable()) {
                event_thread_.join();
            }
            if (external_imu_thread_.joinable()) {
                external_imu_thread_.request_stop();
                external_imu_thread_.join();
            }
        }

        void update() {
            imu_.update_status();
            Eigen::Quaterniond gimbal_imu_pose{imu_.q0(), imu_.q1(), imu_.q2(), imu_.q3()};
        
            if (external_imu_available_.load(std::memory_order::relaxed)) {
                external_imu_.update_status();
                gimbal_imu_pose = gimbal_imu_pose.slerp(0.001, external_imu_.quaternion());
                imu_.q0() = gimbal_imu_pose.w();
                imu_.q1() = gimbal_imu_pose.x();
                imu_.q2() = gimbal_imu_pose.y();
                imu_.q3() = gimbal_imu_pose.z();
            }

            Eigen::Quaterniond foxglove_pose = convertToFoxgloveFrame(gimbal_imu_pose);
            Eigen::Vector3d euler_angles = quaternionToEuler(foxglove_pose);
            
            *gimbal_state_roll_ = euler_angles.x();
            *gimbal_state_pitch_ = euler_angles.y();
            *gimbal_state_yaw_ = euler_angles.z();
            
            double filtered_roll = roll_filter_.update(euler_angles.x());
            double filtered_pitch = pitch_filter_.update(euler_angles.y());
            double filtered_yaw = yaw_filter_.update(euler_angles.z());

            auto compensated = applyDriftCompensation(filtered_roll, filtered_pitch, filtered_yaw);
            
            *gimbal_state_filtered_roll_ = compensated.roll;
            *gimbal_state_filtered_pitch_ = compensated.pitch;
            *gimbal_state_filtered_yaw_ = compensated.yaw;
            
            *quaternion_x_ = foxglove_pose.x();
            *quaternion_y_ = foxglove_pose.y();
            *quaternion_z_ = foxglove_pose.z();
            *quaternion_w_ = foxglove_pose.w();

            RCLCPP_DEBUG_THROTTLE(dart_.get_logger(), *dart_.get_clock(), 1000,
                                "Angles - Raw: [%.3f, %.3f, %.3f], Filtered: [%.3f, %.3f, %.3f]",
                                *gimbal_state_roll_, *gimbal_state_pitch_, *gimbal_state_yaw_,
                                *gimbal_state_filtered_roll_, *gimbal_state_filtered_pitch_, *gimbal_state_filtered_yaw_);
        }

        void command_update() {}
    
    private:
        struct CompensatedAngles {
            double roll;
            double pitch;
            double yaw;
        };

        CompensatedAngles applyDriftCompensation(double filtered_roll, double filtered_pitch, double filtered_yaw) {
            auto current_time = std::chrono::steady_clock::now();
            double elapsed_seconds = std::chrono::duration<double>(current_time - start_time_).count();
            
            if (!calibration_complete_) {
                roll_samples_.push_back({elapsed_seconds, filtered_roll});
                pitch_samples_.push_back({elapsed_seconds, filtered_pitch});
                yaw_samples_.push_back({elapsed_seconds, filtered_yaw});
                
                if (elapsed_seconds >= dart_.first_sample_time_ && !first_sample_recorded_) {
                    first_sample_recorded_ = true;
                    RCLCPP_INFO(dart_.get_logger(), "First sample recorded - Roll: %.6f, Pitch: %.6f, Yaw: %.6f", 
                               filtered_roll, filtered_pitch, filtered_yaw);
                }
                
                if (elapsed_seconds >= dart_.final_sample_time_ && !final_sample_recorded_) {
                    final_sample_recorded_ = true;
                    RCLCPP_INFO(dart_.get_logger(), "Final sample recorded - Roll: %.6f, Pitch: %.6f, Yaw: %.6f", 
                               filtered_roll, filtered_pitch, filtered_yaw);
                    
                    calculateDriftCompensation();
                    calibration_complete_ = true;
                }
                
                return {filtered_roll, filtered_pitch, filtered_yaw};
            } else {
                double compensated_roll = filtered_roll - roll_bias_;
                double compensated_pitch = filtered_pitch - pitch_bias_;
                double compensated_yaw = filtered_yaw - (yaw_drift_slope_ * elapsed_seconds + yaw_bias_);
                
                return {compensated_roll, compensated_pitch, compensated_yaw};
            }
        }
        
        void calculateDriftCompensation() {
            if (!roll_samples_.empty() && !pitch_samples_.empty()) {
                roll_bias_ = roll_samples_.back().value;
                pitch_bias_ = pitch_samples_.back().value;
                
                RCLCPP_INFO(dart_.get_logger(), "Roll bias: %.6f, Pitch bias: %.6f", roll_bias_, pitch_bias_);
            }
            
            if (yaw_samples_.size() < 2) {
                RCLCPP_WARN(dart_.get_logger(), "Not enough yaw samples for drift calculation");
                yaw_drift_slope_ = 0.0;
                yaw_bias_ = 0.0;
                return;
            }
            
            double sum_x = 0.0, sum_y = 0.0, sum_xy = 0.0, sum_xx = 0.0;
            int n = yaw_samples_.size();
            
            for (const auto& sample : yaw_samples_) {
                sum_x += sample.time;
                sum_y += sample.value;
                sum_xy += sample.time * sample.value;
                sum_xx += sample.time * sample.time;
            }
            
            double denominator = n * sum_xx - sum_x * sum_x;
            if (std::abs(denominator) > 1e-10) {
                yaw_drift_slope_ = (n * sum_xy - sum_x * sum_y) / denominator;
                yaw_bias_ = yaw_samples_.back().value - yaw_drift_slope_ * yaw_samples_.back().time;
                
                RCLCPP_INFO(dart_.get_logger(), "Yaw drift slope: k = %.9f rad/s, bias: %.6f", 
                           yaw_drift_slope_, yaw_bias_);
                RCLCPP_INFO(dart_.get_logger(), "Drift compensation activated for all axes");
            } else {
                RCLCPP_WARN(dart_.get_logger(), "Cannot calculate yaw drift slope - singular matrix");
                yaw_drift_slope_ = 0.0;
                yaw_bias_ = yaw_samples_.back().value;
            }
            
            roll_samples_.clear();
            pitch_samples_.clear();
            yaw_samples_.clear();
        }

        static Eigen::Quaterniond convertToFoxgloveFrame(const Eigen::Quaterniond& imu_quat) {
            Eigen::Quaterniond transform;
            transform = Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitZ());
            return transform * imu_quat;
        }

        static Eigen::Vector3d quaternionToEuler(const Eigen::Quaterniond& q) {
            Eigen::Matrix3d rotation_matrix = q.toRotationMatrix();
            
            double roll = std::atan2(rotation_matrix(2, 1), rotation_matrix(2, 2));
            double pitch = std::asin(-rotation_matrix(2, 0));
            double yaw = std::atan2(rotation_matrix(1, 0), rotation_matrix(0, 0));
            
            return {roll, pitch, yaw};
        }

        void can1_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;
        }

        void can2_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;
        }

        void accelerometer_receive_callback(int16_t x, int16_t y, int16_t z) override {
            imu_.store_accelerometer_status(x, y, z);
        }

        void gyroscope_receive_callback(int16_t x, int16_t y, int16_t z) override {
            imu_.store_gyroscope_status(x, y, z);
        }

        void external_imu_thread_main(
            const std::stop_token& stop_token, const std::string& port_name,
            const rclcpp::Logger& logger) {
            try {
                serial::Serial serial{port_name, 115200, serial::Timeout::simpleTimeout(10)};
                rmcs_utility::FpsCounter fps_counter;

                while (!stop_token.stop_requested()) {
                    if (external_imu_.store_status<uint8_t>(serial) && fps_counter.count()) {
                        bool available = fps_counter.fps() > 350.0;
                        if (!available)
                            RCLCPP_WARN(logger, "External IMU low FPS: %.2f", fps_counter.fps());
                        else if (!external_imu_available_.load(std::memory_order::relaxed))
                            RCLCPP_INFO(
                                logger, "External IMU now available with FPS: %.2f", 
                                fps_counter.fps());
                        external_imu_available_.store(available, std::memory_order::relaxed);
                    }  
                }
            } catch (const std::exception& e) {
                external_imu_available_.store(false, std::memory_order::relaxed);
                RCLCPP_ERROR(logger, "Exception in external IMU thread: %s", e.what());
            }
        }

        struct AxisSample {
            double time;    // 时间（秒）
            double value;   // 角度值（弧度）
        };
        
        std::deque<AxisSample> roll_samples_;
        std::deque<AxisSample> pitch_samples_;
        std::deque<AxisSample> yaw_samples_;
        std::chrono::steady_clock::time_point start_time_;
        
        double roll_bias_;
        double pitch_bias_;
        double yaw_drift_slope_;
        double yaw_bias_;
        
        bool calibration_complete_ = false;
        bool first_sample_recorded_ = false;
        bool final_sample_recorded_ = false;

        Dart& dart_;
        OutputInterface<rmcs_description::Tf>& tf_;

        device::Bmi088 imu_;

        OutputInterface<double> gimbal_state_roll_;
        OutputInterface<double> gimbal_state_pitch_;
        OutputInterface<double> gimbal_state_yaw_;
        OutputInterface<double> gimbal_state_filtered_roll_;
        OutputInterface<double> gimbal_state_filtered_pitch_;
        OutputInterface<double> gimbal_state_filtered_yaw_;
        OutputInterface<double> quaternion_x_;
        OutputInterface<double> quaternion_y_;
        OutputInterface<double> quaternion_z_;
        OutputInterface<double> quaternion_w_;

        std::thread event_thread_;

        rmcs_core::hardware::device::Hipnuc external_imu_;
        std::atomic<bool> external_imu_available_ = false;
        std::jthread external_imu_thread_;

        filter::LowPassFilter<1> roll_filter_;
        filter::LowPassFilter<1> pitch_filter_;
        filter::LowPassFilter<1> yaw_filter_;
    };
    
    double first_sample_time_;
    double final_sample_time_;
    OutputInterface<rmcs_description::Tf> tf_;
    std::unique_ptr<TopBoard> top_board_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Dart, rmcs_executor::Component)