#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller {

class DesiredVelocityPublisher final : public rmcs_executor::Component, public rclcpp::Node {
public:
    explicit DesiredVelocityPublisher() noexcept
        : Node{"desired_velocity_publisher", rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)} {
  
        desired_velocity_ = this->get_parameter("desired_velocity").as_double();
        
        RCLCPP_INFO(get_logger(), "Initialized with desired velocity: %f", desired_velocity_);

        register_output("/desired_velocity", velocity_output_, desired_velocity_);
    }
    
    void update() override {
        
        auto& output = *velocity_output_;
        output = desired_velocity_;
    
    }

private:
    double desired_velocity_;
    OutputInterface<double> velocity_output_;
};
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::DesiredVelocityPublisher, rmcs_executor::Component)