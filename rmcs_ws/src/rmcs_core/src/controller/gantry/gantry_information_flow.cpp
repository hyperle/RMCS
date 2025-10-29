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
        , left_velocity_filter_(20.0, 1000.0)
        , right_velocity_filter_(20.0, 1000.0)
        , yaw_velocity_filter_(20.0, 1000.0) {     
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
        set_pid_parameter(left_position_pid_, "left_position");
        set_pid_parameter(right_position_pid_, "right_position");
        set_pid_parameter(yaw_position_pid_, "yaw_position");
        set_pid_parameter(left_velocity_pid_, "left_velocity");
        set_pid_parameter(right_velocity_pid_, "right_velocity");
        set_pid_parameter(yaw_velocity_pid_, "yaw_velocity");
        set_pid_parameter(sync_pid_, "sync");

        position_ff_gain_ = get_parameter("position_ff_gain").as_double();
        velocity_ff_gain_ = get_parameter("velocity_ff_gain").as_double();
        max_torque_ = get_parameter("max_torque").as_double();
        max_sync_compensation_ = get_parameter("max_sync_compensation").as_double();
        oscillation_threshold_count_ = get_parameter("oscillation_threshold_count").as_int();
        
        register_input("/gantry/control/angle",  gantry_control_angle_);
        register_input("/yaw/control/angle",     yaw_control_angle_);
        register_input("/gantry/left/angle",     left_motor_angle_);
        register_input("/gantry/left/velocity",  left_motor_velocity_);
        register_input("/gantry/right/angle",    right_motor_angle_);
        register_input("/gantry/right/velocity", right_motor_velocity_);
        register_input("/gantry/yaw/angle",      yaw_motor_angle_);
        register_input("/gantry/yaw/velocity",   yaw_motor_velocity_);
        
        register_output("/gantry/left/filtered_velocity",  left_motor_filtered_velocity_);
        register_output("/gantry/right/filtered_velocity", right_motor_filtered_velocity_);
        register_output("/gantry/yaw/filtered_velocity",   yaw_motor_filtered_velocity_);
        register_output("/gantry/left/control_torque",     left_motor_control_torque_);
        register_output("/gantry/right/control_torque",    right_motor_control_torque_);
        register_output("/gantry/yaw/control_torque",      yaw_motor_control_torque_);
        
    }

    void update() override {
        if (std::isnan(*gantry_control_angle_) || std::isnan(*yaw_control_angle_)) {
            reset_all_controls();
            return;
        }

        bool new_gantry_input = (std::abs(*gantry_control_angle_ - last_control_angle_) > 0.001);
        bool new_yaw_input = (std::abs(*yaw_control_angle_ - yaw_last_control_angle_) > 0.001);

        last_control_angle_ = *gantry_control_angle_;
        yaw_last_control_angle_ = *yaw_control_angle_;

        if ((new_gantry_input || new_yaw_input) && deadzone_active_) {
            RCLCPP_INFO(get_logger(), "New input detected, exiting deadzone");
            deadzone_active_ = false;
            oscillation_counter_ = 0;
            reset_pid_integrals();
        }

        if (deadzone_active_) {
            reset_all_controls();
            return;
        }

        compute_decoupled_pid_control();
        
        if (detect_oscillation_simple()) {
            RCLCPP_WARN(get_logger(), "Oscillation detected! Activating deadzone");
            deadzone_active_ = true;
            reset_all_controls();
        }
    }
    
private:
    void reset_pid_integrals() {
        left_position_pid_.reset();
        right_position_pid_.reset();
        yaw_position_pid_.reset();
        left_velocity_pid_.reset();
        right_velocity_pid_.reset();
        yaw_velocity_pid_.reset();
        sync_pid_.reset();
    }

    bool detect_oscillation_simple() {
        bool current_oscillation = false;
        
        if ((*left_motor_control_torque_ * last_left_torque_ < 0) || 
            (*right_motor_control_torque_ * last_right_torque_ < 0)) {
            current_oscillation = true;
        }
        
        last_left_torque_ = *left_motor_control_torque_;
        last_right_torque_ = *right_motor_control_torque_;
        last_yaw_torque_ = *yaw_motor_control_torque_;
        
        if (current_oscillation) {
            oscillation_counter_++;
        } else {
            oscillation_counter_ = std::max(0, oscillation_counter_ - 1);
        }
        
        bool oscillation_detected = oscillation_counter_ >= oscillation_threshold_count_;
        
        if (oscillation_detected) {
            RCLCPP_DEBUG(get_logger(), "Oscillation detected: counter=%d", oscillation_counter_);
        }
        
        return oscillation_detected;
    }

    void compute_decoupled_pid_control() {
        process_sensor_data();
        
        Eigen::Vector3d current_angles = {*left_motor_angle_, *right_motor_angle_, *yaw_motor_angle_};
        Eigen::Vector3d filtered_velocities = {*left_motor_filtered_velocity_, *right_motor_filtered_velocity_, *yaw_motor_filtered_velocity_};
        
        double sync_error = calculate_sync_error(current_angles.head<2>());
        
        double target_velocity = calculate_target_velocity(*gantry_control_angle_);
        double position_feedforward = calculate_position_feedforward(*gantry_control_angle_);
        double velocity_feedforward = calculate_velocity_feedforward(target_velocity);

        double left_position_error = *gantry_control_angle_ - current_angles.x();
        double left_velocity_target = left_position_pid_.update(left_position_error);
        
        double left_velocity_error = left_velocity_target - filtered_velocities.x();
        double left_torque = left_velocity_pid_.update(left_velocity_error)
                           + velocity_feedforward
                           + position_feedforward;

        double right_position_error = *gantry_control_angle_ - current_angles.y();
        double right_velocity_target = right_position_pid_.update(right_position_error);
        
        double sync_compensation = sync_pid_.update(-sync_error);
        sync_compensation = std::clamp(sync_compensation, -max_sync_compensation_, max_sync_compensation_);
        right_velocity_target += sync_compensation;
        
        double right_velocity_error = right_velocity_target - filtered_velocities.y();
        double right_torque = right_velocity_pid_.update(right_velocity_error)
                            + velocity_feedforward
                            + position_feedforward;

        double yaw_position_error = *yaw_control_angle_ - current_angles.z();
        double yaw_velocity_target = yaw_position_pid_.update(yaw_position_error);
        
        double yaw_velocity_error = yaw_velocity_target - filtered_velocities.z();
        double yaw_torque = yaw_velocity_pid_.update(yaw_velocity_error);
        
        left_torque = std::clamp(left_torque, -max_torque_, max_torque_);
        right_torque = std::clamp(right_torque, -max_torque_, max_torque_);
        yaw_torque = std::clamp(yaw_torque, -max_torque_, max_torque_);
        
        *left_motor_control_torque_ = left_torque;
        *right_motor_control_torque_ = right_torque;
        *yaw_motor_control_torque_ = yaw_torque;
        
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
                            "Target: %.3f, Torques: [%.3f, %.3f, %.3f], "
                            "SyncErr: %.3f, SyncComp: %.3f, FF: [%.3f, %.3f]",
                            *gantry_control_angle_,
                            *left_motor_control_torque_, *right_motor_control_torque_, *yaw_motor_control_torque_,
                            sync_error, sync_compensation, position_feedforward, velocity_feedforward);
    }

    double calculate_target_velocity(double target_angle) {
        static auto last_time = now();
        auto current_time = now();
        double dt = (current_time - last_time).seconds();
        
        if (dt > 0.0) {
            double velocity = (target_angle - last_control_angle_) / dt;
            last_time = current_time;
            return velocity;
        }
        return 0.0;
    }

    double calculate_position_feedforward(double target_angle) const {
        return position_ff_gain_ * target_angle;
    }

    double calculate_velocity_feedforward(double target_velocity) const {
        return velocity_ff_gain_ * target_velocity;
    }

    void reset_all_controls() {
        *left_motor_control_torque_  = 0.0;
        *right_motor_control_torque_ = 0.0;
        *yaw_motor_control_torque_ = 0.0;
    }

    void process_sensor_data() {
        *left_motor_filtered_velocity_  = left_velocity_filter_.update(*left_motor_velocity_);
        *right_motor_filtered_velocity_ = right_velocity_filter_.update(*right_motor_velocity_);
        *yaw_motor_filtered_velocity_   = yaw_velocity_filter_.update(*yaw_motor_velocity_);
    }

    static double calculate_sync_error(const Eigen::Vector2d& angles) {
        return angles.x() - angles.y(); 
    }

private:
    rclcpp::Logger logger_;
    
    filter::LowPassFilter<1> left_velocity_filter_;
    filter::LowPassFilter<1> right_velocity_filter_;
    filter::LowPassFilter<1> yaw_velocity_filter_;
    
    pid::PidCalculator left_position_pid_, left_velocity_pid_;
    pid::PidCalculator right_position_pid_, right_velocity_pid_;
    pid::PidCalculator yaw_position_pid_, yaw_velocity_pid_;
    pid::PidCalculator sync_pid_;  
    
    double position_ff_gain_;
    double velocity_ff_gain_;
    double max_torque_;
    double max_sync_compensation_;
    
    int oscillation_threshold_count_;
    int oscillation_counter_ = 0;
    double last_left_torque_ = 0.0;
    double last_right_torque_ = 0.0;
    double last_yaw_torque_ = 0.0;
    
    bool deadzone_active_ = false;
    double last_control_angle_;
    double yaw_last_control_angle_;

    InputInterface<double> gantry_control_angle_;
    InputInterface<double> yaw_control_angle_;
    InputInterface<double> left_motor_angle_, left_motor_velocity_;
    InputInterface<double> right_motor_angle_, right_motor_velocity_;
    InputInterface<double> yaw_motor_angle_, yaw_motor_velocity_;

    OutputInterface<double> left_motor_filtered_velocity_;
    OutputInterface<double> right_motor_filtered_velocity_;
    OutputInterface<double> yaw_motor_filtered_velocity_;
    OutputInterface<double> left_motor_control_torque_;
    OutputInterface<double> right_motor_control_torque_;
    OutputInterface<double> yaw_motor_control_torque_;
};

}
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::GantryInformation, rmcs_executor::Component)