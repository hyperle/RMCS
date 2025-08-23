#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <cmath>

namespace rmcs_core::controller {

class SinCosGenerator final : public rmcs_executor::Component, public rclcpp::Node {
public:
    explicit SinCosGenerator() noexcept
        : Node{"sin_cos_generator", rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)} {
        
        this->declare_parameter("omega", 1.0);
        omega_ = this->get_parameter("omega").as_double();
        RCLCPP_INFO(get_logger(), "Initialized with omega: %f", omega_);
        
        register_output("/sin", sin_output_, 0.0);
        register_output("/cos", cos_output_, 0.0);
        
        start_time_ = this->now();
    }
    
    void update() override {
        auto current_time = this->now();
        auto elapsed_time = (current_time - start_time_).seconds();
        
        // 计算sin和cos值
        auto& sin_output = *sin_output_;
        auto& cos_output = *cos_output_;
        
        sin_output = std::sin(omega_ * elapsed_time);
        cos_output = std::cos(omega_ * elapsed_time);
    }

private:
    double omega_;
    rclcpp::Time start_time_;
    OutputInterface<double> sin_output_;
    OutputInterface<double> cos_output_;
};
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::SinCosGenerator, rmcs_executor::Component)
