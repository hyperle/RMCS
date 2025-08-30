#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <cmath>
#include <random>

namespace rmcs_core::controller {

class NoiseSinGenerator final : public rmcs_executor::Component, public rclcpp::Node {
public:
    explicit NoiseSinGenerator() noexcept
        : Node{"noise_sin_generator", rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)} {
  
        omega_ = this->get_parameter("omega").as_double();
        amplitude_ = this->get_parameter("amplitude").as_double();
        RCLCPP_INFO(get_logger(), "Initialized with omega: %f", omega_);
        RCLCPP_INFO(get_logger(), "Initialized with amplitude: %f", amplitude_);
        
        start_time_ = this->now();
        
        register_output("/component/generator/noise_sin", noise_sin_output_, 0.0);
        register_output("/component/generator/random_noise", noise_output_, 0.0);
        register_output("/component/generator/clean_sin", clean_sin_output_, 0.0);
        
        gen_ = std::mt19937(std::random_device{}());
        dist_ = std::uniform_real_distribution<>(-amplitude_, amplitude_);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1), 
            [this]() {
                this->generate_noise();
            });
    }
    
    void update() override {
        auto current_time = this->now();
        auto elapsed_time = (current_time - start_time_).seconds();
     
        double clean_sin = std::sin(omega_ * elapsed_time);
        *clean_sin_output_ = clean_sin;
        
        double current_noise = *noise_output_;
        
        *noise_sin_output_ = clean_sin + current_noise;
    }

private:
    void generate_noise() {
        *noise_output_ = dist_(gen_);
    }

    double amplitude_; 
    double omega_;
    rclcpp::Time start_time_;

    OutputInterface<double> noise_output_;
    OutputInterface<double> noise_sin_output_;
    OutputInterface<double> clean_sin_output_;

    std::mt19937 gen_;
    std::uniform_real_distribution<> dist_;

    rclcpp::TimerBase::SharedPtr timer_;

};
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::NoiseSinGenerator, rmcs_executor::Component)