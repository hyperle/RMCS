#include <cmath>

#include <algorithm>
#include <limits>
#include <numbers>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_utility/eigen_structured_bindings.hpp>

#include "controller/chassis/qcp_solver.hpp"
#include "controller/pid/matrix_pid_calculator.hpp"
#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::controller::chassis {

class GantryController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    GantryController()
        :Node(
            get_component_name(),
            rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , left_position_pid_calculator_(0.0, 0.0, 0.0)
        , left_velocity_pid_calculator_(0.0, 0.0, 0.0)
        , right_position_pid_calculator_(0.0, 0.0, 0.0)
        , right_velocity_pid_calculator_(0.0, 0.0, 0.0) {
        
        register_input("/gantry/left_motor/max_torque", gantry_left_motor_max_control_torque_);
        register_input("/gantry/right_motor/max_torque", gantry_right_motor_max_control_torque_);
        register_input("/gantry/left_motor/velocity", left_motor_velocity_);
        register_input("/gantry/right_motor/velocity", right_motor_velocity_);

        register_output("/gantry/left/motor/position", left_motor_position_, nan_);
        register_output("/gantry/left/motor/position", right_motor_position_, nan_);
        
    }

    void before_updating() override {
        RCLCPP_INFO(
            get_logger(), "Max control torque of left motor: %.f",
            *gantry_left_motor_max_control_torque_);
        RCLCPP_INFO(
            get_logger(), "Max control torque of right motor: %.f",
            *gantry_right_motor_max_control_torque_);
    }

    void update() override {
        if (std::isnan(chassis_control_velocity_->vector[0]))
    }




private:
    static constexpr double inf_ = std::numeric_limits<double>::infinity();
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    InputInterface<double> gantry_left_motor_max_control_torque_;
    InputInterface<double> gantry_right_motor_max_control_torque_;
    InputInterface<double> left_motor_velocity_;
    InputInterface<double> right_motor_velocity_;

    OutputInterface<double> left_motor_position_;
    OutputInterface<double> right_motor_position_;

    pid::PidCalculator left_position_pid_calculator_;
    pid::PidCalculator left_velocity_pid_calculator_;
    pid::PidCalculator right_position_pid_calculator_;
    pid::PidCalculator right_velocity_pid_calculator_;


};





}