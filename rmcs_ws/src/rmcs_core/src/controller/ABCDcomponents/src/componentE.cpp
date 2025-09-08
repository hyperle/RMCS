#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include "filter/learn_filter.hpp"

namespace rmcs_core::controller {

class Filter final : public rmcs_executor::Component, public rclcpp::Node {
public:
    explicit Filter() noexcept
        : Node{"filter", rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)} {

        alpha_ = this->get_parameter("alpha").as_double();
        RCLCPP_INFO(get_logger(), "Initialized with alpha: %f", alpha_);
       
        register_input("/component/generator/noise_sin", noise_sin_input_);
      
        register_output("/filter/lowpass/output", low_pass_output_, 0.0);
        register_output("/filter/median/output", median_output_, 0.0);

        auto& factory = rmcs_core::filter::FilterFactory<1>::instance();

        low_pass_filter_ = std::make_unique<rmcs_core::filter::LowPassFilter<1>>(alpha_);
        RCLCPP_INFO(get_logger(), "LowPassFilter created with alpha: %f", alpha_);
        /*if (auto* low_pass = dynamic_cast<rmcs_core::filter::LowPassFilter<1>*>(low_pass_filter_.get())) {
        low_pass->set_alpha(alpha_);
    }*/

        median_filter_ = factory.create("MedianFilter9");
    }

    void update() override {
 
        const auto& noise_sin_value = *noise_sin_input_;

        auto& low_pass_output = * low_pass_output_;
        auto& median_output = * median_output_;

        low_pass_output = low_pass_filter_->update(noise_sin_value);
        median_output = median_filter_->update(noise_sin_value);
        
        /*double low_pass_filtered = 0.0;
        if (low_pass_filter_) {
            low_pass_filtered = low_pass_filter_->update(noise_sin_value);
        }
        *low_pass_output_ = low_pass_filtered;
        
        double median_filtered = 0.0;
        if (median_filter_) {
            median_filtered = median_filter_->update(noise_sin_value);
        }
        *median_output_ = median_filtered;*/
    }

private:
    double alpha_;

    InputInterface<double> noise_sin_input_;
    OutputInterface<double> low_pass_output_;
    OutputInterface<double> median_output_;

    std::unique_ptr<rmcs_core::filter::Filter<1>> low_pass_filter_;
    std::unique_ptr<rmcs_core::filter::Filter<1>> median_filter_;

    rclcpp::TimerBase::SharedPtr timer_;

};
}


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::Filter, rmcs_executor::Component)