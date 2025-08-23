#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller {

class ComponentB final : public rmcs_executor::Component, public rclcpp::Node {
public:
    explicit ComponentB() noexcept
        : Node{"component_b", rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)} {
        
        RCLCPP_INFO(get_logger(), "Initializing ComponentB");
        
        register_input("/sin", sin_input_);
        register_input("/cos", cos_input_);
      
        register_output("/sum", sum_output_, 0.0);
    }
    
    void update() override {
 
        const auto& sin_value = *sin_input_;
        const auto& cos_value = *cos_input_;
  
        auto& sum_output = *sum_output_;
        sum_output = sin_value + cos_value;
        
        // 可选：记录日志（调试用）
        // RCLCPP_DEBUG(get_logger(), "sin: %f, cos: %f, sum: %f", sin_value, cos_value, sum_output);
    }

private:
    InputInterface<double> sin_input_;
    InputInterface<double> cos_input_;
    OutputInterface<double> sum_output_;
};
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::ComponentB, rmcs_executor::Component)