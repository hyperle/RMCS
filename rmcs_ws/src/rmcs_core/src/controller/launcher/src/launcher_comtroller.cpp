#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include "filter/low_pass_filter.hpp"
#include "controller/pid/pid_calculator.hpp" 
#include "controller/launcher/include/velocity_calculater.hpp"

namespace rmcs_core::controller {

enum class LauncherState : uint8_t {
    LOCKED    = 0,
    UNLOCKED  = 1,
    LOADING   = 2, 
    RESETTING = 3,
    FILLING   = 4,
    HOOKOFF   = 5,
    LAUNCHING = 6,
    ORIGEN    = 7
};

class LauncherInformation
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    LauncherInformation()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger())
        , yaw_velocity_filter_(20.0, 1000.0)
        , launch_velocity_filter_(20.0, 1000.0) {     
        
        auto set_pid_parameter = [this](pid::PidCalculator& pid, const std::string& name) {
            pid.kp = get_parameter(name + "_kp").as_double();
            pid.ki = get_parameter(name + "_ki").as_double();
            pid.kd = get_parameter(name + "_kd").as_double();
            get_parameter(name + "_integral_min",       pid.integral_min);
            get_parameter(name + "_integral_max",       pid.integral_max);
            get_parameter(name + "_integral_split_min", pid.integral_split_min);
            get_parameter(name + "_integral_split_max", pid.integral_split_max);
            get_parameter(name + "_output_min",         pid.output_min);
            get_parameter(name + "_output_max",         pid.output_max);
        };
        
        set_pid_parameter(yaw_angle_pid_, "yaw_angle");
        set_pid_parameter(yaw_velocity_pid_, "yaw_velocity");
        set_pid_parameter(launch_position_pid_, "launch_position");
        set_pid_parameter(launch_velocity_pid_, "launch_velocity");
        
        yaw_torque_oscillation_threshold_ = get_parameter("yaw_torque_oscillation_threshold").as_double();
        yaw_deadzone_duration_ = get_parameter("yaw_deadzone_duration").as_double();
        yaw_oscillation_window_ = get_parameter("yaw_oscillation_window").as_int();
        
        yaw_torque_history_.resize(yaw_oscillation_window_, 0.0);
        
        register_input("/launcher/aim/target", target_);
        register_input("/launcher/control/yaw_angle", yaw_control_angle_);
        register_input("/launcher/control/launch_position", launch_control_position_);
        register_input("/launcher/yaw/angle", yaw_motor_angle_);
        register_input("/launcher/yaw/velocity", yaw_motor_velocity_);
        register_input("/launcher/launch/angle", launch_motor_angle_);
        register_input("/launcher/launch/velocity", launch_motor_velocity_);
        
        register_output("/launcher/yaw/filtered_velocity", yaw_motor_filtered_velocity_);
        register_output("/launcher/launch/filtered_velocity", launch_motor_filtered_velocity_);
        register_output("/launcher/yaw/control_torque", yaw_motor_control_torque_);
        register_output("/launcher/launch/control_torque", launch_motor_control_torque_);
        
        last_yaw_control_angle_ = 0.0;
        last_launch_control_position_ = 0.0;
    }

    void update() override {
        if (std::isnan(*yaw_control_angle_) || std::isnan(*launch_control_position_)) {
            reset_all_controls();
            return;
        }

        bool new_yaw_input = (std::abs(*yaw_control_angle_ - last_yaw_control_angle_) > 0.001);
        bool new_launch_input = (std::abs(*launch_control_position_ - last_launch_control_position_) > 0.001);
        
        last_yaw_control_angle_ = *yaw_control_angle_;
        last_launch_control_position_ = *launch_control_position_;
        
        if ((new_yaw_input || new_launch_input) && yaw_deadzone_active_) {
            RCLCPP_INFO(get_logger(), "New control input detected, exiting yaw deadzone");
            yaw_deadzone_active_ = false;
            yaw_deadzone_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        }

        if (yaw_deadzone_active_) {
            handle_yaw_deadzone();
        } else {
            compute_pid_control();
            update_yaw_torque_history();
            
            if (detect_yaw_torque_oscillation()) {
                activate_yaw_deadzone();
            }
        }
        dart::VelocityCalculator calculator;
        // 调试参数
        calculator.set_spring_constant(250.0);  // 调整弹簧劲度系数
        calculator.set_dart_mass(0.06);         // 调整飞镖质量
    }
    
private:
    void handle_yaw_deadzone() {
        *yaw_motor_control_torque_ = 0.0;
        
        if (yaw_deadzone_start_time_ != rclcpp::Time(0, 0, RCL_ROS_TIME)) {
            double deadzone_elapsed = (now() - yaw_deadzone_start_time_).seconds();
            if (deadzone_elapsed >= yaw_deadzone_duration_) {
                yaw_deadzone_active_ = false;
                RCLCPP_INFO(get_logger(), "Yaw deadzone timeout ended after %.1f seconds", deadzone_elapsed);
            }
        }
    }
    
    void activate_yaw_deadzone() {
        yaw_deadzone_active_ = true;
        yaw_deadzone_start_time_ = now();
        
        RCLCPP_WARN(get_logger(), 
                    "Yaw torque oscillation detected! Activating deadzone for %.1f seconds", 
                    yaw_deadzone_duration_);
        
        *yaw_motor_control_torque_ = 0.0;
    }

    void update_yaw_torque_history() {
        yaw_torque_history_.erase(yaw_torque_history_.begin());
        yaw_torque_history_.push_back(*yaw_motor_control_torque_);
    }
    
    bool detect_yaw_torque_oscillation() {
        int sign_changes = count_sign_changes(yaw_torque_history_);
        double amplitude = calculate_amplitude(yaw_torque_history_);
        
        bool oscillation_detected = (sign_changes >= yaw_oscillation_window_) &&
                                   (amplitude > yaw_torque_oscillation_threshold_);
        
        if (oscillation_detected) {
            RCLCPP_DEBUG(get_logger(), 
                        "Yaw torque oscillation: sign_changes=%d, amplitude=%.2f",
                        sign_changes, amplitude);
        }
        
        return oscillation_detected;
    }
    
    static int count_sign_changes(const std::vector<double>& data) {
        int changes = 0;
        for (size_t i = 1; i < data.size(); ++i) {
            if (data[i] * data[i-1] < 0) {
                changes++;
            }
        }
        return changes;
    }
    
    static double calculate_amplitude(const std::vector<double>& data) {
        if (data.empty()) return 0.0;
        double max_val = *std::max_element(data.begin(), data.end());
        double min_val = *std::min_element(data.begin(), data.end());
        return max_val - min_val;
    }

    void compute_pid_control() {
        process_sensor_data();
        
        double yaw_target_velocity = yaw_angle_pid_.update(*yaw_control_angle_ - *yaw_motor_angle_);
        *yaw_motor_control_torque_ = yaw_velocity_pid_.update(yaw_target_velocity - *yaw_motor_filtered_velocity_);
        
        double launch_target_velocity = launch_position_pid_.update(*launch_control_position_ - *launch_motor_angle_);
        *launch_motor_control_torque_ = launch_velocity_pid_.update(launch_target_velocity - *launch_motor_filtered_velocity_);
        
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
                            "Yaw: target=%.3f, torque=%.3f | Launch: target=%.3f, torque=%.3f",
                            *yaw_control_angle_, *yaw_motor_control_torque_,
                            *launch_control_position_, *launch_motor_control_torque_);
    }

    void reset_all_controls() {//
    }

    void process_sensor_data() {
        *yaw_motor_filtered_velocity_ = yaw_velocity_filter_.update(*yaw_motor_velocity_);
        *launch_motor_filtered_velocity_ = launch_velocity_filter_.update(*launch_motor_velocity_);
    }

private: 
    rclcpp::Logger logger_;
    
    filter::LowPassFilter<1> yaw_velocity_filter_;
    filter::LowPassFilter<1> launch_velocity_filter_;
    
    pid::PidCalculator yaw_angle_pid_, yaw_velocity_pid_;
    pid::PidCalculator launch_position_pid_, launch_velocity_pid_;

    double yaw_torque_oscillation_threshold_;
    double yaw_deadzone_duration_;
    int yaw_oscillation_window_;    
    bool yaw_deadzone_active_ = false;
    rclcpp::Time yaw_deadzone_start_time_;
    std::vector<double> yaw_torque_history_;
    
    double last_yaw_control_angle_;
    double last_launch_control_position_;

    InputInterface<double> target_;
    InputInterface<double> yaw_control_angle_;
    InputInterface<double> launch_control_position_;
    InputInterface<double> yaw_motor_angle_, yaw_motor_velocity_;
    InputInterface<double> launch_motor_angle_, launch_motor_velocity_;

    OutputInterface<double> yaw_motor_filtered_velocity_;
    OutputInterface<double> launch_motor_filtered_velocity_;
    OutputInterface<double> yaw_motor_control_torque_;
    OutputInterface<double> launch_motor_control_torque_;
};

} // namespace rmcs_core::controller

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::LauncherInformation, rmcs_executor::Component)