#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_msgs/switch.hpp>

#include "filter/low_pass_filter.hpp"
#include "controller/pid/pid_calculator.hpp" 

namespace rmcs_core::controller {

class GantryInformation
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    GantryInformation()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger())
        , velocity_filter_(20.0, 1000.0) {     
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
        set_pid_parameter(left_motor_velocity_pid_, "left_motor_angle_to_velocity");
        set_pid_parameter(right_motor_velocity_pid_,"right_motor_angle_to_velocity");
        set_pid_parameter(left_motor_torques_pid_,  "left_motor_velocity_to_torques");
        set_pid_parameter(right_motor_torques_pid_, "right_motor_velocity_to_torques");
        
        torque_oscillation_threshold_ = get_parameter("torque_oscillation_threshold").as_double();
        deadzone_duration_ = get_parameter("deadzone_duration").as_double();
        oscillation_window_ = get_parameter("oscillation_window").as_int();
        
        left_torque_history_.resize(oscillation_window_, 0.0);
        right_torque_history_.resize(oscillation_window_, 0.0);
        
        register_input("/gantry/control/angle",  gantry_control_angle_);
        register_input("/gantry/left/angle",     left_motor_angle_);
        register_input("/gantry/left/velocity",  left_motor_velocity_);
        register_input("/gantry/right/angle",    right_motor_angle_);
        register_input("/gantry/right/velocity", right_motor_velocity_);
        register_output("/gantry/left/filtered_velocity",  left_motor_filtered_velocity_);
        register_output("/gantry/right/filtered_velocity", right_motor_filtered_velocity_);
        register_output("/gantry/left/control_torque",     left_motor_control_torque_);
        register_output("/gantry/right/control_torque",    right_motor_control_torque_);
        
        last_control_angle_ = 0.0;
    }

    void update() override {
        if (std::isnan(*gantry_control_angle_)) {
            reset_all_controls();
            return;
        }

        bool new_joystick_input = (std::abs(*gantry_control_angle_ - last_control_angle_) > 0.001);
        last_control_angle_ = *gantry_control_angle_;
        
        if (new_joystick_input && deadzone_active_) {
            RCLCPP_INFO(get_logger(), "New joystick input detected, exiting deadzone");
            deadzone_active_ = false;
            deadzone_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        }

        if (deadzone_active_) {
            handle_deadzone();
            return;
        }

        compute_pid_control();
        
        update_torque_history();
        if (detect_torque_oscillation()) {
            activate_deadzone();
        }
    }
    
private:
    void handle_deadzone() {
        *left_motor_control_torque_ = 0.0;
        *right_motor_control_torque_ = 0.0;
        
        if (deadzone_start_time_ != rclcpp::Time(0, 0, RCL_ROS_TIME)) {
            double deadzone_elapsed = (now() - deadzone_start_time_).seconds();
            if (deadzone_elapsed >= deadzone_duration_) {
                deadzone_active_ = false;
                RCLCPP_INFO(get_logger(), "Deadzone timeout ended after %.1f seconds", deadzone_elapsed);
            }
        }
    }
    
    void activate_deadzone() {
        deadzone_active_ = true;
        deadzone_start_time_ = now();
        
        RCLCPP_WARN(get_logger(), 
                    "Torque oscillation detected! Activating deadzone for %.1f seconds", 
                    deadzone_duration_);
        
        reset_all_controls();
    }

    void update_torque_history() {
        left_torque_history_.erase(left_torque_history_.begin());
        right_torque_history_.erase(right_torque_history_.begin());
        
        left_torque_history_.push_back(*left_motor_control_torque_);
        right_torque_history_.push_back(*right_motor_control_torque_);
    }
    
    bool detect_torque_oscillation() {
        int left_sign_changes = count_sign_changes(left_torque_history_);
        int right_sign_changes = count_sign_changes(right_torque_history_);
        
        double left_amplitude = calculate_amplitude(left_torque_history_);
        double right_amplitude = calculate_amplitude(right_torque_history_);
        double max_amplitude = std::max(left_amplitude, right_amplitude);
        
        bool oscillation_detected = ((left_sign_changes + right_sign_changes) >= oscillation_window_) &&
                                   (max_amplitude > torque_oscillation_threshold_);
        
        if (oscillation_detected) {
            RCLCPP_DEBUG(get_logger(), 
                        "Torque oscillation: left_changes=%d, right_changes=%d, amplitude=%.2f",
                        left_sign_changes, right_sign_changes, max_amplitude);
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
        
        Eigen::Vector2d current_angles = {*left_motor_angle_, *right_motor_angle_};
        Eigen::Vector2d filtered_velocities = {*left_motor_filtered_velocity_, *right_motor_filtered_velocity_};
        
        double sync_error = calculate_sync_error(current_angles);
        Eigen::Vector2d target_velocities = calculate_target_velocities(current_angles, *gantry_control_angle_);
        Eigen::Vector2d control_torques = calculate_control_torques(filtered_velocities, target_velocities, sync_error);

        *left_motor_control_torque_ = control_torques.x();
        *right_motor_control_torque_ = control_torques.y();
        
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
                            "Target: %.3f, Torques: [%.3f, %.3f]",
                            *gantry_control_angle_,
                            *left_motor_control_torque_, *right_motor_control_torque_);
    }

    void reset_all_controls() {
        *left_motor_control_torque_  = 0.0;
        *right_motor_control_torque_ = 0.0;
    }

    void process_sensor_data() {
        *left_motor_filtered_velocity_  = velocity_filter_.update(*left_motor_velocity_);
        *right_motor_filtered_velocity_ = velocity_filter_.update(*right_motor_velocity_);
    }

    static double calculate_sync_error(const Eigen::Vector2d& angles)  {
        return (angles.x() - angles.y()) / 2; 
    }

    Eigen::Vector2d calculate_target_velocities(const Eigen::Vector2d& current_angles, double target_angle) {
        Eigen::Vector2d target_velocities;
        target_velocities.x() = left_motor_velocity_pid_.update(target_angle - current_angles.x());
        target_velocities.y() = right_motor_velocity_pid_.update(target_angle - current_angles.y());
        return target_velocities;
    }

    Eigen::Vector2d calculate_control_torques(const Eigen::Vector2d& current_velocities, 
                                              const Eigen::Vector2d& target_velocities, 
                                              double sync_error) {
        Eigen::Vector2d control_torques;
        control_torques.x() = left_motor_torques_pid_.update(
            target_velocities.x() - current_velocities.x() - sync_error);
        control_torques.y() = right_motor_torques_pid_.update(
            target_velocities.y() - current_velocities.y() + sync_error);
        return control_torques;
    }

private: 
    rclcpp::Logger logger_;
    filter::LowPassFilter<1> velocity_filter_;
    pid::PidCalculator left_motor_velocity_pid_, left_motor_torques_pid_;
    pid::PidCalculator right_motor_velocity_pid_,right_motor_torques_pid_;

    double torque_oscillation_threshold_;
    double deadzone_duration_;
    int oscillation_window_;    
    bool deadzone_active_ = false;
    rclcpp::Time deadzone_start_time_;
    
    std::vector<double> left_torque_history_;
    std::vector<double> right_torque_history_;
    
    double last_control_angle_;

    InputInterface<double> gantry_control_angle_;
    InputInterface<double> left_motor_angle_, left_motor_velocity_;
    InputInterface<double> right_motor_angle_,right_motor_velocity_;

    OutputInterface<double> left_motor_filtered_velocity_;
    OutputInterface<double> right_motor_filtered_velocity_;
    OutputInterface<double> left_motor_control_torque_;
    OutputInterface<double> right_motor_control_torque_;
};

}
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::GantryInformation, rmcs_executor::Component)