#pragma once

#include <vector>
#include <algorithm>
#include <cmath>
#include <memory>
#include <numbers>
#include <functional>
#include <eigen3/Eigen/Dense>
#include <limits>

namespace rmcs_core::filter {
template <int variable_number = 1>
requires(variable_number > 0) class Filter {
public:
    using Value = std::conditional_t<
        variable_number == 1, 
        double, 
        Eigen::Vector<double, variable_number>
    >;

    virtual ~Filter() = default;
    virtual Value update(const Value& input) = 0;
    virtual void reset() = 0;
    virtual std::unique_ptr<Filter> clone() const = 0;
    virtual const char* type_name() const = 0;

protected:
    static auto get_nan() {
        if constexpr (variable_number == 1) {
            return std::numeric_limits<double>::quiet_NaN();
        }else {
            return Eigen::Vector<double, variable_number>::Constant(
                std::numeric_limits<double>::quiet_NaN()
            );
        }
    }

    static void exclude_nan(double& output,double input) {
        output = std::isnan(output) ? input : output;
    }

    template <int n>
    static void exclude_nan(
        Eigen::Vector<double, n >& output,
        const Eigen::Vector<double, n >& input
    ) {
        output = (output.array().isNaN()).select(input,output);
    }
};


template <int variable_number = 1>
requires(variable_number > 0)
class LowPassFilter : public Filter<variable_number> {
public:
    using Value = typename Filter<variable_number>::Value;

    explicit LowPassFilter(double alpha) 
        : alpha_(alpha)
        , previous_output_(this->get_nan()) {}

    LowPassFilter(const LowPassFilter& other)
        : alpha_(other.alpha_)
        , previous_output_(other.previous_output_) {}

    Value update(const Value& input) override {
        Value output = alpha_ * input + (1.0 - alpha_) * previous_output_;
        this->exclude_nan(output, input);
        previous_output_ = output;
        return output;
    }

    void reset() override { 
        previous_output_ = this->get_nan(); 
    }

    std::unique_ptr<Filter<variable_number>> clone() const override {
        return std::make_unique<LowPassFilter>(*this);
    }

    const char* type_name() const override {
        return "LowPassFilter";
    }

    double get_alpha() const {
        return alpha_;
    }


private:
    double alpha_;
    Value previous_output_; 

};

template <int variable_number = 1, size_t window_size = 5>
requires(variable_number > 0 && window_size > 0) 
class MedianFilter : public Filter<variable_number> {
public:
    using Value = typename Filter<variable_number>::Value;

    MedianFilter() : window_index_(0) {
        reset();
    }

    MedianFilter(const MedianFilter& other)
        : window_(other.window_)
        , window_index_(other.window_index_) {}

    void reset() override {
        window_index_ = 0;
        window_.clear();
        window_.reserve(window_size);
        
        for (size_t i = 0; i < window_size; ++i) {
            window_.push_back(this->get_nan());
        }
    }

    Value update(const Value& input) override {
        window_[window_index_] = input;
        window_index_ = (window_index_ + 1) % window_size;
        
        return calculate_median();
    }

    std::unique_ptr<Filter<variable_number>> clone() const override {
        return std::make_unique<MedianFilter>(*this);
    }

    const char* type_name() const override {
        return "MedianFilter";
    }

    size_t get_window_size() const {
        return window_size;
    }


private:
    Value calculate_median() {
        if constexpr (variable_number == 1) {
            std::vector<double> valid_values;
            valid_values.reserve(window_size);
            
            for (const auto& value : window_) {
                if (!std::isnan(value)) {
                    valid_values.push_back(value);
                }
            }
            
            if (valid_values.empty()) {
                return this->get_nan();
            }
            
            std::sort(valid_values.begin(), valid_values.end());
            size_t mid = valid_values.size() / 2;
            
            if (valid_values.size() % 2 == 0) {
                return (valid_values[mid - 1] + valid_values[mid]) / 2.0;
            } else {
                return valid_values[mid];
            }
        } else {
            Value result;
            
            for (int i = 0; i < variable_number; ++i) {
                std::vector<double> component_values;
                component_values.reserve(window_size);
                
                for (const auto& value : window_) {
                    if (!std::isnan(value[i])) {
                        component_values.push_back(value[i]);
                    }
                }
                
                if (component_values.empty()) {
                    result[i] = std::numeric_limits<double>::quiet_NaN();
                    continue;
                }
                
                std::sort(component_values.begin(), component_values.end());
                size_t mid = component_values.size() / 2;
                
                if (component_values.size() % 2 == 0) {
                    result[i] = (component_values[mid - 1] + component_values[mid]) / 2.0;
                } else {
                    result[i] = component_values[mid];
                }
            }
            
            return result;
        }
    }

    std::vector<Value> window_;
    size_t window_index_; 
};

template <int variable_number = 1>
class FilterFactory {
public:
    using FilterPtr = std::unique_ptr<Filter<variable_number>>;
    using CreatorFunc = std::function<FilterPtr()>;
 
    static FilterFactory& instance() {
        static FilterFactory instance;
        return instance;
    }
    
    void register_creator(const std::string& type_name, CreatorFunc creator) {
        creators_[type_name] = std::move(creator);
    }
  
    FilterPtr create(const std::string& type_name) const {
        auto it = creators_.find(type_name);
        if (it != creators_.end()) {
            return it->second();
        }
        return nullptr;
    }
    
    std::vector<std::string> available_types() const {
        std::vector<std::string> types;
        for (const auto& pair : creators_) {
            types.push_back(pair.first);
        }
        return types;
    }

private:
    FilterFactory() {
        register_creator("LowPassFilter", []() {
            return std::make_unique<LowPassFilter<variable_number>>(0.1);
        });
        
        register_creator("MedianFilter9", []() {
            return std::make_unique<MedianFilter<variable_number, 9>>();
        });
    }
    
    std::unordered_map<std::string, CreatorFunc> creators_;
};

using ScalarLowPassFilter = LowPassFilter<1>;
using ScalarMedianFilter = MedianFilter<1, 9>;

using Vector3dLowPassFilter = LowPassFilter<3>;
using Vector3dMedianFilter = MedianFilter<3, 5>;

using ScalarFilterFactory = FilterFactory<1>;
using Vector3dFilterFactory = FilterFactory<3>;

} // namespace rmcs_core::filter