#include <atomic>
#include <memory>
#include <thread>
#include <deque>
#include <Eigen/Dense>

#include <librmcs/client/cboard.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_utility/fps_counter.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "hardware/device/bmi088.hpp"

namespace rmcs_core::hardware {

class KalmanFilter {
public:
    explicit KalmanFilter(double dt = 0.001, double process_noise = 1e-5, double measurement_noise = 1e-3) 
        : dt_(dt), process_noise_(process_noise), measurement_noise_(measurement_noise) {
        
        x_ = Eigen::Vector2d::Zero();
        
        F_ = Eigen::Matrix2d::Identity();
        F_(0, 1) = dt_;
        
        H_ = Eigen::Matrix<double, 1, 2>::Zero();
        H_(0, 0) = 1.0;
        
        Q_ = Eigen::Matrix2d::Identity() * process_noise;
        
        R_ = Eigen::Matrix<double, 1, 1>::Identity() * measurement_noise;
        
        P_ = Eigen::Matrix2d::Identity();
    }
    
    void init(double angle, double angular_velocity = 0.0) {
        x_ << angle, angular_velocity;
        P_ = Eigen::Matrix2d::Identity();
    }
    
    double update(double measurement) {
        x_ = F_ * x_;
        P_ = F_ * P_ * F_.transpose() + Q_;
        
        Eigen::Matrix<double, 1, 1> y = Eigen::Matrix<double, 1, 1>(measurement - H_ * x_);
        Eigen::Matrix<double, 1, 1> S = H_ * P_ * H_.transpose() + R_;
        Eigen::Matrix<double, 2, 1> K = P_ * H_.transpose() * S.inverse();
        
        x_ = x_ + K * y;
        P_ = (Eigen::Matrix2d::Identity() - K * H_) * P_;
        
        return x_(0);
    }
    
    double getAngle() const { return x_(0); }
    double getAngularVelocity() const { return x_(1); }

private:
    double dt_;
    double process_noise_;
    double measurement_noise_;
    
    Eigen::Vector2d x_;
    Eigen::Matrix2d F_;
    Eigen::Matrix<double, 1, 2> H_;
    Eigen::Matrix2d P_;
    Eigen::Matrix2d Q_;
    Eigen::Matrix<double, 1, 1> R_;
};

class Dart 
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    Dart()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , command_component_(
              create_partner_component<DartCommand>(get_component_name() + "_command", *this)) {
        
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
            , roll_kalman_(0.001, 1e-5, 1e-3)  // dt, process_noise, measurement_noise
            , pitch_kalman_(0.001, 1e-5, 1e-3)
            , yaw_kalman_(0.001, 1e-5, 1e-3) {

            dart.register_output("/gimbal/state/roll", gimbal_state_roll_);
            dart.register_output("/gimbal/state/pitch", gimbal_state_pitch_);
            dart.register_output("/gimbal/state/yaw", gimbal_state_yaw_);
            dart.register_output("/gimbal/state/filtered_roll", gimbal_state_filtered_roll_);
            dart.register_output("/gimbal/state/filtered_pitch", gimbal_state_filtered_pitch_);
            dart.register_output("/gimbal/state/filtered_yaw", gimbal_state_filtered_yaw_);
            dart.register_output("/gimbal/state/kalman_roll", gimbal_state_kalman_roll_);
            dart.register_output("/gimbal/state/kalman_pitch", gimbal_state_kalman_pitch_);
            dart.register_output("/gimbal/state/kalman_yaw", gimbal_state_kalman_yaw_);

            event_thread_ = std::thread([this]() { handle_events(); });
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(dart_);
            start_time_ = std::chrono::steady_clock::now();
            calibration_complete_ = false;
            kalman_initialized_ = false;
        }

        ~TopBoard() override {
            stop_handling_events();
            if (event_thread_.joinable()) {
                event_thread_.join();
            }
        }

        void update() {
            imu_.update_status();
            Eigen::Quaterniond gimbal_imu_pose{imu_.q0(), imu_.q1(), imu_.q2(), imu_.q3()};

            tf_->set_transform<rmcs_description::PitchLink, rmcs_description::OdomImu>(
                gimbal_imu_pose.conjugate());

            Eigen::Vector3d euler_angles = quaternionToEuler(gimbal_imu_pose);
            
            *gimbal_state_roll_ = euler_angles.x();
            *gimbal_state_pitch_ = euler_angles.y();
            *gimbal_state_yaw_ = euler_angles.z();
            
            if (!kalman_initialized_) {
                roll_kalman_.init(euler_angles.x());
                pitch_kalman_.init(euler_angles.y());
                yaw_kalman_.init(euler_angles.z());
                kalman_initialized_ = true;
            }
            
            double kalman_roll = roll_kalman_.update(euler_angles.x());
            double kalman_pitch = pitch_kalman_.update(euler_angles.y());
            double kalman_yaw = yaw_kalman_.update(euler_angles.z());
            
            *gimbal_state_kalman_roll_ = kalman_roll;
            *gimbal_state_kalman_pitch_ = kalman_pitch;
            *gimbal_state_kalman_yaw_ = kalman_yaw;
            
            auto compensated = applyDriftCompensation(kalman_roll, kalman_pitch, kalman_yaw);
            
            *gimbal_state_filtered_roll_ = compensated.roll;
            *gimbal_state_filtered_pitch_ = compensated.pitch;
            *gimbal_state_filtered_yaw_ = compensated.yaw;

            tf_->set_state<rmcs_description::YawLink, rmcs_description::PitchLink>(compensated.yaw);

            publishTfTransforms(gimbal_imu_pose, compensated.yaw, compensated.pitch);

            RCLCPP_DEBUG_THROTTLE(dart_.get_logger(), *dart_.get_clock(), 1000,
                                "Angles - Raw: [%.3f, %.3f, %.3f], Kalman: [%.3f, %.3f, %.3f], Compensated: [%.3f, %.3f, %.3f]",
                                *gimbal_state_roll_, *gimbal_state_pitch_, *gimbal_state_yaw_,
                                *gimbal_state_kalman_roll_, *gimbal_state_kalman_pitch_, *gimbal_state_kalman_yaw_,
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
                    RCLCPP_INFO(dart_.get_logger(), "First sample recorded");
                }
                
                if (elapsed_seconds >= dart_.final_sample_time_ && !final_sample_recorded_) {
                    final_sample_recorded_ = true;
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
            }
            
            if (yaw_samples_.size() < 2) {
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
            } else {
                yaw_drift_slope_ = 0.0;
                yaw_bias_ = yaw_samples_.back().value;
            }
            
            roll_samples_.clear();
            pitch_samples_.clear();
            yaw_samples_.clear();
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

        // 添加 TF 发布方法
        void publishTfTransforms(const Eigen::Quaterniond& imu_quaternion, double yaw_angle, double pitch_angle) {
            // 创建 TF 广播器（如果尚未创建）
            static std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ = 
                std::make_unique<tf2_ros::TransformBroadcaster>(dart_);
            
            auto now = dart_.get_clock()->now();
            
            // 发布 base_link -> gimbal_center_link 变换
            geometry_msgs::msg::TransformStamped base_to_gimbal;
            base_to_gimbal.header.stamp = now;
            base_to_gimbal.header.frame_id = "base_link";
            base_to_gimbal.child_frame_id = "gimbal_center_link";
            base_to_gimbal.transform.translation.x = 0.0;
            base_to_gimbal.transform.translation.y = 0.0;
            base_to_gimbal.transform.translation.z = 0.1; // 假设云台中心在基座上方10cm
            base_to_gimbal.transform.rotation.x = 0.0;
            base_to_gimbal.transform.rotation.y = 0.0;
            base_to_gimbal.transform.rotation.z = 0.0;
            base_to_gimbal.transform.rotation.w = 1.0;
            tf_broadcaster_->sendTransform(base_to_gimbal);
            
            // 发布 gimbal_center_link -> yaw_link 变换
            geometry_msgs::msg::TransformStamped gimbal_to_yaw;
            gimbal_to_yaw.header.stamp = now;
            gimbal_to_yaw.header.frame_id = "gimbal_center_link";
            gimbal_to_yaw.child_frame_id = "yaw_link";
            gimbal_to_yaw.transform.translation.x = 0.0;
            gimbal_to_yaw.transform.translation.y = 0.0;
            gimbal_to_yaw.transform.translation.z = 0.0;
            
            // Yaw 旋转（绕 Z 轴）- 正确转换为四元数
            Eigen::AngleAxisd yaw_axis(yaw_angle, Eigen::Vector3d::UnitZ());
            Eigen::Quaterniond yaw_quat(yaw_axis);
            gimbal_to_yaw.transform.rotation.x = yaw_quat.x();
            gimbal_to_yaw.transform.rotation.y = yaw_quat.y();
            gimbal_to_yaw.transform.rotation.z = yaw_quat.z();
            gimbal_to_yaw.transform.rotation.w = yaw_quat.w();
            tf_broadcaster_->sendTransform(gimbal_to_yaw);
            
            // 发布 yaw_link -> pitch_link 变换
            geometry_msgs::msg::TransformStamped yaw_to_pitch;
            yaw_to_pitch.header.stamp = now;
            yaw_to_pitch.header.frame_id = "yaw_link";
            yaw_to_pitch.child_frame_id = "pitch_link";
            yaw_to_pitch.transform.translation.x = 0.0;
            yaw_to_pitch.transform.translation.y = 0.0;
            yaw_to_pitch.transform.translation.z = 0.05; // 假设 Pitch 关节在 Yaw 关节上方5cm
            
            // Pitch 旋转（绕 Y 轴）- 正确转换为四元数
            Eigen::AngleAxisd pitch_axis(pitch_angle, Eigen::Vector3d::UnitY());
            Eigen::Quaterniond pitch_quat(pitch_axis);
            yaw_to_pitch.transform.rotation.x = pitch_quat.x();
            yaw_to_pitch.transform.rotation.y = pitch_quat.y();
            yaw_to_pitch.transform.rotation.z = pitch_quat.z();
            yaw_to_pitch.transform.rotation.w = pitch_quat.w();
            tf_broadcaster_->sendTransform(yaw_to_pitch);
            
            // 发布 pitch_link -> odom_imu 变换
            geometry_msgs::msg::TransformStamped pitch_to_imu;
            pitch_to_imu.header.stamp = now;
            pitch_to_imu.header.frame_id = "pitch_link";
            pitch_to_imu.child_frame_id = "odom_imu";
            pitch_to_imu.transform.translation.x = 0.0;
            pitch_to_imu.transform.translation.y = 0.0;
            pitch_to_imu.transform.translation.z = 0.0;
            
            // 使用 IMU 四元数
            pitch_to_imu.transform.rotation.x = imu_quaternion.x();
            pitch_to_imu.transform.rotation.y = imu_quaternion.y();
            pitch_to_imu.transform.rotation.z = imu_quaternion.z();
            pitch_to_imu.transform.rotation.w = imu_quaternion.w();
            tf_broadcaster_->sendTransform(pitch_to_imu);
            
            // 发布固定坐标系（世界坐标系）
            geometry_msgs::msg::TransformStamped world_to_base;
            world_to_base.header.stamp = now;
            world_to_base.header.frame_id = "world";
            world_to_base.child_frame_id = "base_link";
            world_to_base.transform.translation.x = 0.0;
            world_to_base.transform.translation.y = 0.0;
            world_to_base.transform.translation.z = 0.0;
            world_to_base.transform.rotation.x = 0.0;
            world_to_base.transform.rotation.y = 0.0;
            world_to_base.transform.rotation.z = 0.0;
            world_to_base.transform.rotation.w = 1.0;
            tf_broadcaster_->sendTransform(world_to_base);
}

        struct AxisSample {
            double time;
            double value;
        };
        
        std::deque<AxisSample> roll_samples_;
        std::deque<AxisSample> pitch_samples_;
        std::deque<AxisSample> yaw_samples_;
        std::chrono::steady_clock::time_point start_time_;
        
        double roll_bias_ = 0.0;
        double pitch_bias_ = 0.0;
        double yaw_drift_slope_ = 0.0;
        double yaw_bias_ = 0.0;
        
        bool calibration_complete_ = false;
        bool first_sample_recorded_ = false;
        bool final_sample_recorded_ = false;
        bool kalman_initialized_ = false;

        Dart& dart_;
        OutputInterface<rmcs_description::Tf>& tf_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        device::Bmi088 imu_;

        OutputInterface<double> gimbal_state_roll_;
        OutputInterface<double> gimbal_state_pitch_;
        OutputInterface<double> gimbal_state_yaw_;
        OutputInterface<double> gimbal_state_filtered_roll_;
        OutputInterface<double> gimbal_state_filtered_pitch_;
        OutputInterface<double> gimbal_state_filtered_yaw_;
        OutputInterface<double> gimbal_state_kalman_roll_;
        OutputInterface<double> gimbal_state_kalman_pitch_;
        OutputInterface<double> gimbal_state_kalman_yaw_;

        std::thread event_thread_;

        KalmanFilter roll_kalman_;
        KalmanFilter pitch_kalman_;
        KalmanFilter yaw_kalman_;
    };
    
    double first_sample_time_;
    double final_sample_time_;
    OutputInterface<rmcs_description::Tf> tf_;
    std::unique_ptr<TopBoard> top_board_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Dart, rmcs_executor::Component)