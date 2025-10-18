/*#include <memory>
#include <cmath>
#include <algorithm>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "controller/pid/pid_calculator.hpp"
#include "hardware/device/lk_motor.hpp" 

namespace rmcs_core::hardware {

class GantryController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    GantryController()
        : Node(get_component_name(), rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {
        
        this->declare_parameter("gantry_sync_tolerance", 2.0);
        this->declare_parameter("gantry_sync_gain", 0.5);
        this->declare_parameter("gantry_max_tilt_angle", 5.0);
        this->declare_parameter("gantry_emergency_stop_threshold", 10.0);
        
        this->declare_parameter("left_position_kp", 2.0);
        this->declare_parameter("left_position_ki", 0.05);
        this->declare_parameter("left_position_kd", 0.1);
        this->declare_parameter("right_position_kp", 2.0);
        this->declare_parameter("right_position_ki", 0.05);
        this->declare_parameter("right_position_kd", 0.1);
        this->declare_parameter("left_velocity_kp", 1.0);
        this->declare_parameter("left_velocity_ki", 0.02);
        this->declare_parameter("left_velocity_kd", 0.005);
        this->declare_parameter("right_velocity_kp", 1.0);
        this->declare_parameter("right_velocity_ki", 0.02);
        this->declare_parameter("right_velocity_kd", 0.005);

        declare_parameter("left_motor_zero_point", 0);
        declare_parameter("right_motor_zero_point", 0);
        declare_parameter("gantry_max_velocity", 200.0);

        register_output("/gantry/left_position", left_position_output_, 0.0);
        register_output("/gantry/right_position", right_position_output_, 0.0);
        register_output("/gantry/sync_error", sync_error_output_);
        register_output("/gantry/safety_status", safety_status_output_);

        gantry_command_subscription_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/gantry/command", rclcpp::QoS{10},
            [this](std_msgs::msg::Float64MultiArray::UniquePtr msg) {
                gantry_command_callback(std::move(msg));
            });

        parameters_callback_handle_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>& parameters) {
                return on_parameter_changed(parameters);
            });

        initialize_motors();
        initialize_pid_controllers();
    }

    void update() override {
        update_motor_status();
        read_encoder_data();
        check_safety_status();
        update_outputs();
    }

    void command_update() {
        if (!safety_ok_) {
            stop_motors();
            return;
        }
        
        calculate_control_output();
        send_motor_commands();
    }

private:
    class SyncMonitor {
    public:
        struct SyncStatus {
            double sync_error;
            double tilt_angle;
            bool is_synchronized;
            bool is_critical;
        };
        
        SyncStatus check_sync(double left_pos, double right_pos, 
                             double sync_tolerance, double max_tilt_angle,
                             double emergency_threshold) {
            SyncStatus status;
            status.sync_error = left_pos - right_pos;
            
            const double gantry_width = 1000.0;
            status.tilt_angle = std::atan2(status.sync_error, gantry_width) * 180.0 / M_PI;
            
            status.is_synchronized = std::abs(status.sync_error) <= sync_tolerance;
            status.is_critical = std::abs(status.sync_error) >= emergency_threshold;
            
            return status;
        }
    };

    void initialize_motors() {
        rmcs_core::hardware::device::LkMotor::Config left_config{rmcs_core::hardware::device::LkMotor::Type::MG4010E_I10};
        left_config.set_encoder_zero_point(get_parameter("left_motor_zero_point").as_int());
        
        rmcs_core::hardware::device::LkMotor::Config right_config{rmcs_core::hardware::device::LkMotor::Type::MG4010E_I10};
        right_config.set_encoder_zero_point(get_parameter("right_motor_zero_point").as_int());
        
        left_motor_.configure(left_config);
        right_motor_.configure(right_config);
    }


    void initialize_pid_controllers() {
        auto left_pos_kp = get_parameter("left_position_kp").as_double();
        auto left_pos_ki = get_parameter("left_position_ki").as_double();
        auto left_pos_kd = get_parameter("left_position_kd").as_double();
        auto left_vel_kp = get_parameter("left_velocity_kp").as_double();
        auto left_vel_ki = get_parameter("left_velocity_ki").as_double();
        auto left_vel_kd = get_parameter("left_velocity_kd").as_double();
        
        auto right_pos_kp = get_parameter("right_position_kp").as_double();
        auto right_pos_ki = get_parameter("right_position_ki").as_double();
        auto right_pos_kd = get_parameter("right_position_kd").as_double();
        auto right_vel_kp = get_parameter("right_velocity_kp").as_double();
        auto right_vel_ki = get_parameter("right_velocity_ki").as_double();
        auto right_vel_kd = get_parameter("right_velocity_kd").as_double();
        
        left_position_pid_ = std::make_unique<rmcs_core::controller::pid::PidCalculator>(left_pos_kp, left_pos_ki, left_pos_kd);
        left_velocity_pid_ = std::make_unique<rmcs_core::controller::pid::PidCalculator>(left_vel_kp, left_vel_ki, left_vel_kd);
        right_position_pid_ = std::make_unique<rmcs_core::controller::pid::PidCalculator>(right_pos_kp, right_pos_ki, right_pos_kd);
        right_velocity_pid_ = std::make_unique<rmcs_core::controller::pid::PidCalculator>(right_vel_kp, right_vel_ki, right_vel_kd);
    }

    rcl_interfaces::msg::SetParametersResult on_parameter_changed(const std::vector<rclcpp::Parameter>& parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        
        for (const auto& param : parameters) {
            if (param.get_name() == "left_position_kp") left_position_pid_->kp = param.as_double();
            else if (param.get_name() == "left_position_ki") left_position_pid_->ki = param.as_double();
            else if (param.get_name() == "left_position_kd") left_position_pid_->kd = param.as_double();
            else if (param.get_name() == "right_position_kp") right_position_pid_->kp = param.as_double();
            else if (param.get_name() == "right_position_ki") right_position_pid_->ki = param.as_double();
            else if (param.get_name() == "right_position_kd") right_position_pid_->kd = param.as_double();
            else if (param.get_name() == "left_velocity_kp") left_velocity_pid_->kp = param.as_double();
            else if (param.get_name() == "left_velocity_ki") left_velocity_pid_->ki = param.as_double();
            else if (param.get_name() == "left_velocity_kd") left_velocity_pid_->kd = param.as_double();
            else if (param.get_name() == "right_velocity_kp") right_velocity_pid_->kp = param.as_double();
            else if (param.get_name() == "right_velocity_ki") right_velocity_pid_->ki = param.as_double();
            else if (param.get_name() == "right_velocity_kd") right_velocity_pid_->kd = param.as_double();
        }
        
        return result;
    }

    void gantry_command_callback(std_msgs::msg::Float64MultiArray::UniquePtr msg) {
        if (msg->data.size() >= 1) {
            target_position_ = msg->data[0];
            left_position_pid_->reset();
            right_position_pid_->reset();
        }
    }

    void update_motor_status() {
        left_motor_.update_status();
        right_motor_.update_status();
    }

    void read_encoder_data() {
        current_left_position_ = left_motor_.angle();
        current_right_position_ = right_motor_.angle();
        current_left_velocity_ = left_motor_.velocity();
        current_right_velocity_ = right_motor_.velocity();
        
        left_limit_switch_ = read_left_limit_switch();
        right_limit_switch_ = read_right_limit_switch();
        upper_limit_switch_ = read_upper_limit_switch();
        lower_limit_switch_ = read_lower_limit_switch();
        
        apply_limit_protection();
    }

    void apply_limit_protection() {
        const double soft_limit_margin = 10.0;
        
        if (upper_limit_switch_ || current_left_position_ >= upper_limit_ - soft_limit_margin) {
            current_left_position_ = std::min(current_left_position_, upper_limit_ - soft_limit_margin);
            if (target_position_ > current_left_position_) target_position_ = current_left_position_;
        }
        if (upper_limit_switch_ || current_right_position_ >= upper_limit_ - soft_limit_margin) {
            current_right_position_ = std::min(current_right_position_, upper_limit_ - soft_limit_margin);
            if (target_position_ > current_right_position_) target_position_ = current_right_position_;
        }
        
        if (lower_limit_switch_ || current_left_position_ <= lower_limit_ + soft_limit_margin) {
            current_left_position_ = std::max(current_left_position_, lower_limit_ + soft_limit_margin);
            if (target_position_ < current_left_position_) target_position_ = current_left_position_;
        }
        if (lower_limit_switch_ || current_right_position_ <= lower_limit_ + soft_limit_margin) {
            current_right_position_ = std::max(current_right_position_, lower_limit_ + soft_limit_margin);
            if (target_position_ < current_right_position_) target_position_ = current_right_position_;
        }
    }

    void check_safety_status() {
        auto sync_tolerance = get_parameter("gantry_sync_tolerance").as_double();
        auto max_tilt_angle = get_parameter("gantry_max_tilt_angle").as_double();
        auto emergency_threshold = get_parameter("gantry_emergency_stop_threshold").as_double();
        auto sync_status = sync_monitor_.check_sync(
            current_left_position_, current_right_position_,
            sync_tolerance, max_tilt_angle, emergency_threshold);
        
        current_sync_error_ = sync_status.sync_error;
        current_tilt_angle_ = sync_status.tilt_angle;
        
        safety_ok_ = !sync_status.is_critical && std::abs(sync_status.tilt_angle) <= max_tilt_angle;
        
        if (sync_status.is_critical) {
            emergency_stop_triggered_ = true;
            reset_pid_controllers();
        }
    }

    void calculate_control_output() {
        if (emergency_stop_triggered_) {
            left_control_output_ = 0;
            right_control_output_ = 0;
            return;
        }
        
        auto left_position_error = target_position_ - current_left_position_;
        auto right_position_error = target_position_ - current_right_position_;
        
        apply_sync_compensation(left_position_error, right_position_error);
        
        auto left_target_velocity = left_position_pid_->update(left_position_error);
        auto right_target_velocity = right_position_pid_->update(right_position_error);
        
        auto max_velocity = get_parameter("gantry_max_velocity").as_double();
        left_target_velocity = std::clamp(left_target_velocity, -max_velocity, max_velocity);
        right_target_velocity = std::clamp(right_target_velocity, -max_velocity, max_velocity);
        
        auto left_velocity_error = left_target_velocity - current_left_velocity_;
        auto right_velocity_error = right_target_velocity - current_right_velocity_;
        
        left_control_output_ = left_velocity_pid_->update(left_velocity_error);
        right_control_output_ = right_velocity_pid_->update(right_velocity_error);
    }

    void apply_sync_compensation(double& left_error, double& right_error) {
        auto sync_gain = this->get_parameter("gantry_sync_gain").as_double();
        auto sync_tolerance = this->get_parameter("gantry_sync_tolerance").as_double();
        
        if (std::abs(current_sync_error_) > sync_tolerance) {
            double compensation = sync_gain * current_sync_error_;
            
            if (current_sync_error_ > 0) {
                left_error -= compensation;
                right_error += compensation;
            } else {
                left_error += compensation;
                right_error -= compensation;
            }
        }
    }

    void send_motor_commands() {
        left_control_output_ = std::clamp(left_control_output_, -1.0, 1.0);
        right_control_output_ = std::clamp(right_control_output_, -1.0, 1.0);
        
        auto left_command = left_motor_.generate_velocity_command(left_control_output_);
        auto right_command = right_motor_.generate_velocity_command(right_control_output_);
        
    }

    void stop_motors() {
    left_control_output_ = 0;
    right_control_output_ = 0;
    
    // 直接生成停止命令
    auto left_command = left_motor_.generate_velocity_command(0);
    auto right_command = right_motor_.generate_velocity_command(0);
    
    send_can_command(left_command, right_command);
    
    reset_pid_controllers();
}

    void reset_pid_controllers() {
        left_position_pid_->reset();
        right_position_pid_->reset();
        left_velocity_pid_->reset();
        right_velocity_pid_->reset();
    }

    void update_outputs() {
        *left_position_output_ = current_left_position_;
        *right_position_output_ = current_right_position_;
        *sync_error_output_ = current_sync_error_;
        *safety_status_output_ = safety_ok_ ? 1.0 : 0.0;
    }
    
    static bool read_left_limit_switch() { return false; }
    static bool read_right_limit_switch() { return false; }
    static bool read_upper_limit_switch() { return false; }
    static bool read_lower_limit_switch() { return false; }

    rmcs_core::hardware::device::LkMotor left_motor_;
    rmcs_core::hardware::device::LkMotor right_motor_;

    std::unique_ptr<rmcs_core::controller::pid::PidCalculator> left_position_pid_;
    std::unique_ptr<rmcs_core::controller::pid::PidCalculator> right_position_pid_;
    std::unique_ptr<rmcs_core::controller::pid::PidCalculator> left_velocity_pid_;
    std::unique_ptr<rmcs_core::controller::pid::PidCalculator> right_velocity_pid_;
    
    SyncMonitor sync_monitor_;
    
    double target_position_;
    double current_left_position_;
    double current_right_position_;
    double current_left_velocity_;
    double current_right_velocity_;
    double current_sync_error_;
    double current_tilt_angle_;
    double left_control_output_;
    double right_control_output_;
    
    bool safety_ok_;
    bool emergency_stop_triggered_;
    
    const double upper_limit_ = 1000.0;
    const double lower_limit_ = 0.0;
    bool left_limit_switch_;
    bool right_limit_switch_;
    bool upper_limit_switch_;
    bool lower_limit_switch_;
    
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr gantry_command_subscription_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;

    OutputInterface<double> left_position_output_;
    OutputInterface<double> right_position_output_;
    OutputInterface<double> sync_error_output_;
    OutputInterface<double> safety_status_output_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::GantryController, rmcs_executor::Component)







#include <iostream>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <memory>
#include "../armor_detector/detector/minimum_jerk.hpp"

namespace rmcs_core::controller {

class MinimumJerk final : public rmcs_executor::Component, public rclcpp::Node {
public:
    explicit MinimumJerk() noexcept
        : Node{"sin_cos_generator", rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)} {
  
        max_velocity_ = this->get_parameter("max_velocity").as_double();
        max_acceleration_ = this->get_parameter("max_acceleration").as_double();
        waypoints_ = toScalar(get_parameter("waypoints").as_integer_array());
        
        register_input("/component/minimum_jerk/max_velocity", sin_output_, 0.0);
        register_intput("/component/minimum_jerk/max_acceleration", cos_output_, 0.0);
        register_intput("/component/minimum_jerk/waypoints", cos_output_, 0.0);

        start_time_ = this->now();
    }
    
    void update() override {
        auto current_time = this->now();
        auto elapsed_time = (current_time - start_time_).seconds();
     
        auto& sin_output = *sin_output_;
        auto& cos_output = *cos_output_;
        
        sin_output = std::sin(omega_ * elapsed_time);
        cos_output = std::cos(omega_ * elapsed_time);
    }

private:
    double max_velocity_;
    double max_acceleration_;
    
    rclcpp::Time start_time_;
    OutputInterface<double> sin_output_;
    OutputInterface<double> cos_output_;
};
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::MinimumJerk, rmcs_executor::Component)*/
