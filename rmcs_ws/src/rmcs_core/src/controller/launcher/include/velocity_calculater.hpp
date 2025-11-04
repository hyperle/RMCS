#pragma once

#include <cmath>
#include <Eigen/Dense>
#include <rmcs_description/tf_description.hpp>

namespace rmcs_core::controller::dart {
using namespace rmcs_description;

enum class Target : uint8_t {
    OutPost  = 0,
    Base     = 1,
    Static   = 2,
    Switch   = 3,
    Random   = 4,
    Terminal = 5
};

enum class ControlMode : uint8_t {
    None = 0,
    TraceAutoAdjust  = 1,
    TerminalManeuver = 2,
    Loitering  = 3
};

class VelocityCalculator {
public:
    VelocityCalculator() {
        gravity_ = 9.8;  
        dart_mass_ = 0.05; 
        spring_constant_ = 200.0;  
        launch_height_ = 0.5; 
        max_stretch_ = 0.3;  
        energy_efficiency_ = 0.7;  
        air_resistance_coeff_ = 0.01;  
    }

    auto calculate_velocity(Target target, ControlMode control_mode, double distance, double height) {
        switch_target_to_quantity(target, control_mode, distance, height);
        
        double required_velocity = calculate_required_velocity(distance, height);
        double stretch_distance = velocity_to_stretch(required_velocity);
        
        apply_control_mode_adjustment(control_mode, stretch_distance, required_velocity);
        
        return std::make_pair(required_velocity, stretch_distance);
    }

    void set_spring_constant(double k) { spring_constant_ = k; }
    void set_dart_mass(double mass) { dart_mass_ = mass; }
    void set_energy_efficiency(double efficiency) { energy_efficiency_ = efficiency; }
    void set_max_stretch(double max_stretch) { max_stretch_ = max_stretch; }
    void set_air_resistance_coeff(double coeff) { air_resistance_coeff_ = coeff; }

private:
    static void switch_target_to_quantity(Target target, ControlMode& control_mode, double& distance, double& height) {
        switch (target) {//need to be adjusted
            case Target::OutPost: 
                distance = 10.0;
                height = 0.1;
                control_mode = ControlMode::TraceAutoAdjust;
                break;
            case Target::Static:
                distance = 15.0;
                height = 0.1;
                control_mode = ControlMode::None;
                break;
            case Target::Switch:
                distance = 12.0;
                height = 0.1;
                control_mode = ControlMode::TraceAutoAdjust;
                break;
            case Target::Random:
                distance = 8.0;
                height = 0.1;
                control_mode = ControlMode::TerminalManeuver;
                break;
            case Target::Terminal:
                distance = 20.0;
                height = 0.1;
                control_mode = ControlMode::Loitering;
                break;
            default:
                break;
        }
    }

    double calculate_required_velocity(double distance, double target_height) {
        double height_difference = target_height - launch_height_;
        double optimal_angle = M_PI / 4.0; 
        
        double velocity = calculate_velocity_with_air_resistance(distance, height_difference, optimal_angle);
        
        return velocity;
    }

    double calculate_velocity_with_air_resistance(double distance, double height_diff, double angle) {
        
        double v0 = 10.0;  // 初始猜测速度
        double tolerance = 0.01;
        double max_iterations = 100;
        
        for (int i = 0; i < max_iterations; ++i) {
            double current_range = calculate_range(v0, angle, height_diff);
            double error = distance - current_range;
            
            if (std::abs(error) < tolerance) {
                break;
            }
            
            v0 += error * 0.1;
            v0 = std::max(v0, 5.0);  // 最小速度限制
            v0 = std::min(v0, 50.0); // 最大速度限制
        }
        
        return v0;
    }

    double calculate_range(double v0, double angle, double height_diff) const {
        double vx = v0 * std::cos(angle);
        double vy = v0 * std::sin(angle);
        
        double dt = 0.01;
        double x = 0, y = launch_height_;
        double vx_current = vx, vy_current = vy;
        
        while (y > 0) { 
            double speed = std::sqrt(vx_current * vx_current + vy_current * vy_current);
            double drag_force = air_resistance_coeff_ * speed * speed;
            
            double drag_accel_x = drag_force * std::cos(angle) / dart_mass_;
            double drag_accel_y = drag_force * std::sin(angle) / dart_mass_;
            
            // 更新速度（考虑重力和阻力）
            vx_current -= drag_accel_x * dt;
            vy_current -= (gravity_ + drag_accel_y) * dt;
            
            // 更新位置
            x += vx_current * dt;
            y += vy_current * dt;
        }
        
        return x;
    }

    double velocity_to_stretch(double velocity) const {
        // 飞镖动能
        double kinetic_energy = 0.5 * dart_mass_ * velocity * velocity;
        // 考虑能量损失后的所需弹性势能
        double required_spring_energy = kinetic_energy / energy_efficiency_;
        // 两根弹簧并联的总劲度系数
        double total_spring_constant = 2.0 * spring_constant_;
        // 弹性势能公式: E = 0.5 * k * x^2
        double stretch_distance = std::sqrt(2.0 * required_spring_energy / total_spring_constant);
        // 限制在最大拉伸范围内
        stretch_distance = std::min(stretch_distance, max_stretch_);
        
        return stretch_distance;
    }

    static void apply_control_mode_adjustment(ControlMode mode, double& stretch_distance, double& velocity) {
        switch (mode) {
            case ControlMode::TraceAutoAdjust:
                // 追踪自动调整：增加10%的余量
                stretch_distance *= 1.1;
                velocity *= 1.1;
                break;
            case ControlMode::TerminalManeuver:
                // 末端机动：增加速度变化能力
                stretch_distance *= 1.15;
                velocity *= 1.15;
                break;
            case ControlMode::Loitering:
                // 游弋模式：减小能量消耗
                stretch_distance *= 0.9;
                velocity *= 0.9;
                break;
            case ControlMode::None:
            default:
                break;
        }
    }

    double gravity_;
    double dart_mass_;
    double spring_constant_; 
    double launch_height_;
    double max_stretch_;
    double energy_efficiency_;
    double air_resistance_coeff_;
};

}  // namespace rmcs_core::controller::dart