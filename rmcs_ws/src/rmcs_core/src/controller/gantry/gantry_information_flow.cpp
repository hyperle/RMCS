#include <numbers>
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
        *left_motor_filtered_velocity_  = velocity_filter_.update(*left_motor_velocity_);
        *right_motor_filtered_velocity_ = velocity_filter_.update(*right_motor_velocity_);
        Eigen::Vector2d gantry_angle = {*left_motor_angle_, *right_motor_angle_};
        Eigen::Vector2d gantry_velocity = {*left_motor_filtered_velocity_, *right_motor_filtered_velocity_};
        
        Eigen::Vector2d control_velocity_ = calculate_control_velocity(gantry_angle,*gantry_control_angle_);
        Eigen::Vector2d control_torques_ = calculate_control_torques(gantry_velocity, control_velocity_, capture_leftandright_angle_err(gantry_angle));

        *left_motor_control_torque_ = control_torques_.x();
        *right_motor_control_torque_ = control_torques_.y();

    }

    
    void reset_all_controls() {
        *left_motor_control_torque_ = 0.0;
        *right_motor_control_torque_ = 0.0;
    }

    bool is_control_valid() const {
        return !std::isnan(*gantry_control_angle_);
    }

    static double capture_leftandright_angle_err(const Eigen::Vector2d& gantry_angle){
        double angle_err=gantry_angle.x()-gantry_angle.y();
        return angle_err;
    }

    static double place_translate_to_angle(double place){
        double angle;
        angle = place*(2*std::numbers::pi /6);
        return angle;
    }



    Eigen::Vector2d calculate_control_velocity(const Eigen::Vector2d& gantry_angle,const double& control_angle){
        Eigen::Vector2d pid_velocity;
        pid_velocity.x() = left_motor_velocity_pid_.update(control_angle-gantry_angle.x());
        pid_velocity.y() = left_motor_torques_pid_.update(control_angle-gantry_angle.y());
        return pid_velocity;
    }

    Eigen::Vector2d calculate_control_torques(const Eigen::Vector2d& gantry_velocity,Eigen::Vector2d control_velocity,const double& angle_err){
        Eigen::Vector2d pid_torques;
        pid_torques.x() = left_motor_torques_pid_.update(control_velocity.x()-gantry_velocity.x()-angle_err);
        pid_torques.y() = right_motor_torques_pid_.update(control_velocity.y()-gantry_velocity.y()+angle_err);
        return pid_torques;
    }

private:
        rclcpp::Logger logger_;
        filter::LowPassFilter<1> velocity_filter_;
        pid::PidCalculator left_motor_velocity_pid_, left_motor_torques_pid_;
        pid::PidCalculator right_motor_velocity_pid_,right_motor_torques_pid_;
        InputInterface<double> gantry_control_angle_;
        InputInterface<double> left_motor_angle_, left_motor_velocity_;
        InputInterface<double> right_motor_angle_, right_motor_velocity_;
        OutputInterface<double> left_motor_filtered_velocity_;
        OutputInterface<double> right_motor_filtered_velocity_;
        OutputInterface<double> left_motor_control_torque_;
        OutputInterface<double> right_motor_control_torque_;

};

}