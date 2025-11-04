/*#include <memory>
#include <thread>
#include <vector>
#include <numeric>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <librmcs/client/cboard.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "hardware/device/bmi088.hpp"

namespace rmcs_core::hardware {

class Dart : public rmcs_executor::Component, public rclcpp::Node {
public:
    Dart()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , command_component_(create_partner_component<DartCommand>(get_component_name() + "_command", *this)) {

        register_output("/tf", tf_);

        imu_sensitivity_   = this->get_parameter("imu_sensitivity").as_double();
        first_sample_spot_ = this->get_parameter("first_sample_spot").as_double();
        final_sample_spot_ = this->get_parameter("final_sample_spot").as_double();

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        top_board_ = std::make_unique<TopBoard>(*this, *command_component_,
                    static_cast<int>(get_parameter("usb_pid_top_board").as_int()));
    }

    ~Dart() override = default;

    void update() override { top_board_->update(); }
    void command_update() { top_board_->command_update(); }

private:
    class DartCommand : public rmcs_executor::Component {
    public:
        explicit DartCommand(Dart& dart) : dart_(dart) {}
        void update() override { dart_.command_update(); }
        Dart& dart_;
    };
    std::shared_ptr<DartCommand> command_component_;

    class TopBoard final : private librmcs::client::CBoard {
    public:
        explicit TopBoard(Dart& dart, DartCommand& dart_command, int usb_pid = -1)
            : librmcs::client::CBoard(usb_pid)
            , dart_(dart)
            , tf_(dart.tf_)
            , tf_broadcaster_(dart.tf_broadcaster_)
            , imu_(1000, 0.2, 0.0)
            , imu_sens_(dart.imu_sensitivity_)
            , first_s_(dart.first_sample_spot_)
            , final_s_(dart.final_sample_spot_)
            , quaternion_filter_(15.0, 1000.0)  // 四元数贝叶斯滤波
            , quaternion_smoother_(5.0, 1000.0) // 后处理平滑滤波
            {

            registerOutputs();
            event_thread_ = std::thread([this]() { handle_events(); });
            initialize();
        }

        ~TopBoard() override {
            stop_handling_events();
            if (event_thread_.joinable()) event_thread_.join();
        }

        void update() {
            imu_.update_status();
            Eigen::Quaterniond raw_quaternion{
                imu_.q0() / imu_sens_, imu_.q1() / imu_sens_, 
                imu_.q2() / imu_sens_, imu_.q3() / imu_sens_
            };
            raw_quaternion.normalize();
            processImuData(raw_quaternion);
        }

        void command_update() {}

    private:
        class QuaternionBayesFilter {
        public:
            QuaternionBayesFilter(double confidence_ratio, double sample_rate)
                : alpha_(confidence_ratio / (confidence_ratio + 1.0))  // 贝叶斯置信度参数
                , initialized_(false) {}

            Eigen::Quaterniond update(const Eigen::Quaterniond& measurement) {
                if (!initialized_) {
                    belief_ = measurement;
                    initialized_ = true;
                    return belief_;
                }
                
                belief_ = belief_.slerp(1.0 - alpha_, measurement);
                return belief_;
            }
            
            void reset() { initialized_ = false; }
            double getConfidence() const { return alpha_; }

        private:
            double alpha_;  // 置信度参数，越大表示对先验越信任
            bool initialized_;
            Eigen::Quaterniond belief_;  // 当前信念状态
        };

        void registerOutputs() {
            dart_.register_output("/gimbal/state/roll", gimbal_state_roll_);
            dart_.register_output("/gimbal/state/pitch", gimbal_state_pitch_);
            dart_.register_output("/gimbal/state/yaw", gimbal_state_yaw_);
            dart_.register_output("/gimbal/state/final_roll", gimbal_state_filtered_roll_);
            dart_.register_output("/gimbal/state/final_pitch", gimbal_state_filtered_pitch_);
            dart_.register_output("/gimbal/state/filtered_yaw", gimbal_state_filtered_yaw_);
            dart_.register_output("/gimbal/state/final_yaw", gimbal_state_final_yaw_);
        }

        void initialize() {
            start_time_ = std::chrono::steady_clock::now();
            calibration_complete_ = false;
            sample_counter_ = 0;
            quaternion_filter_.reset();
            quaternion_smoother_.reset();
        }

        void processImuData(const Eigen::Quaterniond& raw_quaternion) {
            Eigen::Quaterniond bayes_quaternion = quaternion_filter_.update(raw_quaternion);
            
            Eigen::Quaterniond smoothed_quaternion = quaternion_smoother_.update(bayes_quaternion);
            
            Eigen::Vector3d filtered_euler = quaternionToEuler(bayes_quaternion);
            Eigen::Vector3d smoothed_euler = quaternionToEuler(smoothed_quaternion);
            
            auto compensated = applyDriftCompensation(smoothed_euler);
            
            setOutputStates(filtered_euler, smoothed_euler, compensated);
            setTfTransforms(smoothed_quaternion, compensated);
            logDebugInfo(filtered_euler, smoothed_euler, compensated);
        }

        struct CompensatedResult { double roll, pitch, yaw; };
        
        CompensatedResult applyDriftCompensation(const Eigen::Vector3d& angles) {
            auto current_time = std::chrono::steady_clock::now();
            double elapsed_seconds = std::chrono::duration<double>(current_time - start_time_).count();
            
            if (!calibration_complete_) {
                collectCalibrationSamples(angles, elapsed_seconds);
                return {angles.x(), angles.y(), angles.z()};
            } else {
                double compensated_roll = angles.x() - roll_bias_;
                double compensated_pitch = angles.y() - pitch_bias_;
                double compensated_yaw = angles.z() - (yaw_drift_slope_ * elapsed_seconds + yaw_drift_offset_);
                return {compensated_roll, compensated_pitch, compensated_yaw};
            }
        }

        void collectCalibrationSamples(const Eigen::Vector3d& angles, double elapsed_seconds) {
            if (sample_counter_ % 100 == 0) {  // 1000Hz，每100次取样一次
                if (elapsed_seconds >= first_s_ && elapsed_seconds <= final_s_) {
                    roll_samples_.push_back(angles.x());
                    pitch_samples_.push_back(angles.y());
                    yaw_samples_.push_back(angles.z());
                    time_samples_.push_back(elapsed_seconds);
                }
            }
            sample_counter_++;

            if (elapsed_seconds >= final_s_ && !calibration_complete_) {
                calculateCompensationParameters();
                calibration_complete_ = true;
                RCLCPP_INFO(dart_.get_logger(), "Calibration complete with %zu samples", roll_samples_.size());
                clearSamples();
            }
        }

        void calculateCompensationParameters() {
            if (roll_samples_.empty()) return;

            roll_bias_ = std::accumulate(roll_samples_.begin(), roll_samples_.end(), 0.0) / roll_samples_.size();
            pitch_bias_ = std::accumulate(pitch_samples_.begin(), pitch_samples_.end(), 0.0) / pitch_samples_.size();
            
            calculateYawDriftCompensation();
            
            RCLCPP_INFO(dart_.get_logger(), "Bias - Roll: %.6f, Pitch: %.6f, Yaw slope: %.6f", 
                       roll_bias_, pitch_bias_, yaw_drift_slope_);
        }

        void calculateYawDriftCompensation() {
            if (yaw_samples_.size() < 2) return;

            double sum_x = 0.0, sum_y = 0.0, sum_xy = 0.0, sum_xx = 0.0;
            int n = yaw_samples_.size();
            
            for (size_t i = 0; i < n; ++i) {
                sum_x += time_samples_[i];
                sum_y += yaw_samples_[i];
                sum_xy += time_samples_[i] * yaw_samples_[i];
                sum_xx += time_samples_[i] * time_samples_[i];
            }
            
            double denominator = n * sum_xx - sum_x * sum_x;
            if (std::abs(denominator) > 1e-10) {
                yaw_drift_slope_ = (n * sum_xy - sum_x * sum_y) / denominator;
                yaw_drift_offset_ = (sum_y - yaw_drift_slope_ * sum_x) / n;
            }
        }

        void clearSamples() {
            roll_samples_.clear();
            pitch_samples_.clear();
            yaw_samples_.clear();
            time_samples_.clear();
        }

        void setOutputStates(const Eigen::Vector3d& filtered, const Eigen::Vector3d& smoothed, 
                           const CompensatedResult& compensated) {
            *gimbal_state_roll_ = filtered.x();
            *gimbal_state_pitch_ = filtered.y();
            *gimbal_state_yaw_ = filtered.z();
            
            *gimbal_state_filtered_roll_ = smoothed.x();
            *gimbal_state_filtered_pitch_ = smoothed.y();
            *gimbal_state_filtered_yaw_ = smoothed.z();
        
            *gimbal_state_final_yaw_ = compensated.yaw;
        }

        void setTfTransforms(const Eigen::Quaterniond& imu_quaternion, const CompensatedResult& compensated) {
            tf_->set_transform<rmcs_description::PitchLink, rmcs_description::OdomImu>(imu_quaternion.conjugate());
            tf_->set_state<rmcs_description::YawLink, rmcs_description::PitchLink>(compensated.yaw);
            publishTfTransforms(imu_quaternion, compensated.yaw, compensated.pitch);
        }

        void publishTfTransforms(const Eigen::Quaterniond& imu_quaternion, double yaw_angle, double pitch_angle) {
            auto now = dart_.get_clock()->now();
            
            auto create_transform = [&](const std::string& parent, const std::string& child, 
                                      const geometry_msgs::msg::Vector3& trans, 
                                      const geometry_msgs::msg::Quaternion& rot) {
                geometry_msgs::msg::TransformStamped tf;
                tf.header.stamp = now;
                tf.header.frame_id = parent;
                tf.child_frame_id = child;
                tf.transform.translation = trans;
                tf.transform.rotation = rot;
                return tf;
            };

            geometry_msgs::msg::Vector3 zero_trans = create_translation(0, 0, 0);
            geometry_msgs::msg::Vector3 pitch_trans = create_translation(0, 0, 0.05);
            
            // base_link -> gimbal_center_link
            tf_broadcaster_->sendTransform(create_transform("base_link", "gimbal_center_link", 
                zero_trans, create_rotation(0, 0, 0, 1)));
            
            // gimbal_center_link -> yaw_link
            Eigen::AngleAxisd yaw_axis(yaw_angle, Eigen::Vector3d::UnitZ());
            tf_broadcaster_->sendTransform(create_transform("gimbal_center_link", "yaw_link", 
                zero_trans, create_rotation(Eigen::Quaterniond(yaw_axis))));
            
            // yaw_link -> pitch_link  
            Eigen::AngleAxisd pitch_axis(pitch_angle, Eigen::Vector3d::UnitY());
            tf_broadcaster_->sendTransform(create_transform("yaw_link", "pitch_link", 
                pitch_trans, create_rotation(Eigen::Quaterniond(pitch_axis))));
            
            // pitch_link -> odom_imu
            tf_broadcaster_->sendTransform(create_transform("pitch_link", "odom_imu", 
                zero_trans, create_rotation(imu_quaternion)));
            
            // world -> base_link
            tf_broadcaster_->sendTransform(create_transform("world", "base_link", 
                zero_trans, create_rotation(0, 0, 0, 1)));
        }

        void logDebugInfo(const Eigen::Vector3d& filtered, const Eigen::Vector3d& smoothed, 
                         const CompensatedResult& compensated) {
            static size_t counter = 0;
            if (++counter % 500 == 0) {
                RCLCPP_DEBUG(dart_.get_logger(), 
                    "Bayes[%.3f,%.3f,%.3f] Smooth[%.3f,%.3f,%.3f] Final[%.3f,%.3f,%.3f]",
                    filtered.x(), filtered.y(), filtered.z(),
                    smoothed.x(), smoothed.y(), smoothed.z(),
                    compensated.roll, compensated.pitch, compensated.yaw);
            }
        }

        static Eigen::Vector3d quaternionToEuler(const Eigen::Quaterniond& q) {
            Eigen::Matrix3d m = q.toRotationMatrix();
            return {std::atan2(m(2,1), m(2,2)), std::asin(-m(2,0)), std::atan2(m(1,0), m(0,0))};
        }

        static geometry_msgs::msg::Vector3 create_translation(double x, double y, double z) {
            geometry_msgs::msg::Vector3 t; t.x = x; t.y = y; t.z = z; return t;
        }

        static geometry_msgs::msg::Quaternion create_rotation(double x, double y, double z, double w) {
            geometry_msgs::msg::Quaternion r; r.x = x; r.y = y; r.z = z; r.w = w; return r;
        }

        static geometry_msgs::msg::Quaternion create_rotation(const Eigen::Quaterniond& q) {
            return create_rotation(q.x(), q.y(), q.z(), q.w());
        }

        void accelerometer_receive_callback(int16_t x, int16_t y, int16_t z) override {
            x = static_cast<int16_t>(x / imu_sens_);
            y = static_cast<int16_t>(y / imu_sens_);
            z = static_cast<int16_t>(z / imu_sens_);
            imu_.store_accelerometer_status(x, y, z);
        }

        void gyroscope_receive_callback(int16_t x, int16_t y, int16_t z) override {
            x = static_cast<int16_t>(x / imu_sens_);
            y = static_cast<int16_t>(y / imu_sens_);
            z = static_cast<int16_t>(z / imu_sens_);
            imu_.store_gyroscope_status(x, y, z);
        }

        Dart& dart_;
        OutputInterface<rmcs_description::Tf>& tf_;
        std::unique_ptr<tf2_ros::TransformBroadcaster>& tf_broadcaster_;

        device::Bmi088 imu_;
        double imu_sens_, first_s_, final_s_;

        OutputInterface<double> gimbal_state_roll_, gimbal_state_pitch_, gimbal_state_yaw_;
        OutputInterface<double> gimbal_state_filtered_roll_, gimbal_state_filtered_pitch_, gimbal_state_filtered_yaw_;
        OutputInterface<double> gimbal_state_final_yaw_;

        std::thread event_thread_;

        QuaternionBayesFilter quaternion_filter_;  
        QuaternionBayesFilter quaternion_smoother_; 

        bool calibration_complete_ = false;
        std::chrono::steady_clock::time_point start_time_;
        size_t sample_counter_ = 0;
        
        std::vector<double> roll_samples_, pitch_samples_, yaw_samples_, time_samples_;
        double roll_bias_ = 0.0, pitch_bias_ = 0.0, yaw_drift_slope_ = 0.0, yaw_drift_offset_ = 0.0;
    };

    double imu_sensitivity_, first_sample_spot_, final_sample_spot_;
    OutputInterface<rmcs_description::Tf> tf_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<TopBoard> top_board_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Dart, rmcs_executor::Component)*/