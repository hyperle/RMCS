#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_msgs/switch.hpp>

#include "filter/low_pass_filter.hpp"
#include "controller/pid/pid_calculator.hpp" 



namespace rmcs_core::controller{

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
        register_input("/gantry/left/angle",     left_motor_angle_);
        register_input("/gantry/left/velocity",  left_motor_velocity_);
        register_input("/gantry/right/angle",    right_motor_angle_);
        register_input("/gantry/right/velocity", right_motor_velocity_);
        register_input("/gantry/control_angle",  gantry_control_angle_);
        register_output("/gantry/left/filtered_velocity",  left_motor_filtered_velocity_);
        register_output("/gantry/right/filtered_velocity", right_motor_filtered_velocity_);
        register_output("/gantry/left/control_torque",     left_motor_control_torque_);
        register_output("/gantry/right/control_torque",    right_motor_control_torque_);
    }

    void update() override {
        if (!is_control_valid()) {
            reset_all_controls();
            return;
        }

        process_sensor_data();
        compute_control_outputs();

    }

    void reset_all_controls() {
        *left_motor_control_torque_ = 0.0;
        *right_motor_control_torque_ = 0.0;
    }

    bool is_control_valid() const {
        return !std::isnan(*gantry_control_angle_);
    }

    void process_sensor_data() {
        *left_motor_filtered_velocity_  = velocity_filter_.update(*left_motor_velocity_);
        *right_motor_filtered_velocity_ = velocity_filter_.update(*right_motor_velocity_);
    }

    void compute_control_outputs() {
        Eigen::Vector2d current_angles = {*left_motor_angle_, *right_motor_angle_};
        Eigen::Vector2d filtered_velocities = {*left_motor_filtered_velocity_, *right_motor_filtered_velocity_};
        
        double sync_error = calculate_sync_error(current_angles);
        
        Eigen::Vector2d target_velocities = calculate_target_velocities(current_angles, *gantry_control_angle_);
        Eigen::Vector2d control_torques = calculate_control_torques(filtered_velocities, target_velocities, sync_error);

        *left_motor_control_torque_ = control_torques.x();
        *right_motor_control_torque_ = control_torques.y();
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