#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_msgs/switch.hpp>

#include "filter/low_pass_filter.hpp"
#include "controller/pid/pid_calculator.hpp" 

namespace rmcs_core::controller {

class GantryInformation
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    GantryInformation()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger())
        , left_velocity_filter_(20.0, 1000.0)
        ,right_velocity_filter_(20.0, 1000.0)
        ,yaw_velocity_filter_(20.0, 1000.0) {     
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
        set_pid_parameter(yaw_motor_velocity_pid_,  "yaw_motor_angle_to_velocity");
        set_pid_parameter(left_motor_torques_pid_,  "left_motor_velocity_to_torques");
        set_pid_parameter(right_motor_torques_pid_, "right_motor_velocity_to_torques");
        set_pid_parameter(yaw_motor_torques_pid_,   "yaw_motor_velocity_to_torques");
        
        
        gantry_dead_zone_duration_ = get_parameter("dead_zone_duration").as_double();
        yaw_dead_zone_duration_ = get_parameter("dead_zone_duration").as_double();
        gantry_oscillation_threshold_ = get_parameter("gantry_oscillation_threshold").as_double();
        yaw_oscillation_threshold_ = get_parameter("yaw_oscillation_threshold").as_double();
        oscillation_window_ = get_parameter("oscillation_window").as_int();
        
        left_torque_history_.resize(oscillation_window_, 0.0);
        right_torque_history_.resize(oscillation_window_, 0.0);
        yaw_torque_history_.resize(oscillation_window_, 0.0);
        
        sync_gain_ = get_parameter("sync_gain").as_double();
        position_ff_gain_ = get_parameter("position_feedforward_gain").as_double();
        velocity_ff_gain_ = get_parameter("velocity_feedforward_gain").as_double();
        load_compensation_gain_ = get_parameter("load_compensation_gain").as_double();
        max_torque_ = get_parameter("max_torque").as_double();
        max_sync_compensation_= get_parameter("max_sync_compensation").as_double();
        
        register_input("/gantry/control/angle",  gantry_control_angle_);
        register_input("/yaw/control/angle",     yaw_control_angle_);
        register_input("/gantry/left/angle",     left_motor_angle_);
        register_input("/gantry/left/velocity",  left_motor_velocity_);
        register_input("/gantry/right/angle",    right_motor_angle_);
        register_input("/gantry/right/velocity", right_motor_velocity_);
        register_input("/gantry/yaw/angle",      yaw_motor_angle_);
        register_input("/gantry/yaw/velocity",   yaw_motor_velocity_);
        register_output("/gantry/left/filtered_velocity",  left_motor_filtered_velocity_);
        register_output("/gantry/right/filtered_velocity", right_motor_filtered_velocity_);
        register_output("/gantry/yaw/filtered_velocity",   yaw_motor_filtered_velocity_);
        register_output("/gantry/left/control_torque",     left_motor_control_torque_);
        register_output("/gantry/right/control_torque",    right_motor_control_torque_);
        register_output("/gantry/yaw/control_torque",      yaw_motor_control_torque_);
        
        last_control_angle_ = 0.0;
        yaw_last_control_angle_ = 0.0;
    }

    void update() override {
        if (std::isnan(*gantry_control_angle_)) {
            reset_all_controls();
            return;
        }

        if (std::isnan(*yaw_control_angle_)) {
            reset_all_controls();
            return;
        }


        bool new_gantry_input = (std::abs(*gantry_control_angle_ - last_control_angle_) > 0.001);
        bool new_yaw_input = (std::abs(*yaw_control_angle_ - yaw_last_control_angle_) > 0.001);

        last_control_angle_ = *gantry_control_angle_;
        yaw_last_control_angle_ = *yaw_control_angle_;

        if (new_gantry_input && gantry_dead_zone_active_) {
            RCLCPP_INFO(get_logger(), "New gantry input detected, exiting gantry deadzone");
            gantry_dead_zone_active_ = false;
        }

        if (new_yaw_input && yaw_dead_zone_active_) {
            RCLCPP_INFO(get_logger(), "New yaw input detected, exiting yaw deadzone");
            yaw_dead_zone_active_ = false;
        }

        if (gantry_dead_zone_active_ || yaw_dead_zone_active_) {
            if (gantry_dead_zone_active_) {
                reset_gantry_controls();
            }
            if (yaw_dead_zone_active_) {
                reset_yaw_controls();
            }
            
            update_torque_history();
            detect_and_activate_deadzones();
            return;
        }

        compute_pid_control();
        detect_and_activate_deadzones();
    }
    
private:
    void activate_gantry_deadzone() {
        gantry_dead_zone_active_ = true;
        reset_gantry_controls();
        RCLCPP_WARN(get_logger(), "Gantry torque oscillation detected! Activating gantry deadzone");
    }
    
    void activate_yaw_deadzone() {
        yaw_dead_zone_active_ = true;
        reset_yaw_controls();
        RCLCPP_WARN(get_logger(), "Yaw torque oscillation detected! Activating yaw deadzone");
    }

    void update_torque_history() {
        left_torque_history_.erase(left_torque_history_.begin());
        right_torque_history_.erase(right_torque_history_.begin());
        yaw_torque_history_.erase(yaw_torque_history_.begin());
        
        left_torque_history_.push_back(*left_motor_control_torque_);
        right_torque_history_.push_back(*right_motor_control_torque_);
        yaw_torque_history_.push_back(*yaw_motor_control_torque_);
    }
    
    bool detect_gantry_oscillation() {
        if (left_torque_history_.size() < 3 || right_torque_history_.size() < 3) {
            return false;
        }
        
        bool left_oscillating = detect_single_motor_oscillation(left_torque_history_, gantry_oscillation_threshold_);
        bool right_oscillating = detect_single_motor_oscillation(right_torque_history_, gantry_oscillation_threshold_);
        
        bool oscillation_detected = left_oscillating || right_oscillating;
        
        if (oscillation_detected) {
            RCLCPP_DEBUG(get_logger(), 
                        "Gantry oscillation detected: left=%d, right=%d", 
                        left_oscillating, right_oscillating);
        }
        
        return oscillation_detected;
    }

    bool detect_yaw_oscillation() {
        if (yaw_torque_history_.size() < 3) {
            return false;
        }
        
        bool yaw_oscillating = detect_single_motor_oscillation(yaw_torque_history_, yaw_oscillation_threshold_);
        
        if (yaw_oscillating) {
            RCLCPP_DEBUG(get_logger(), "Yaw oscillation detected");
        }
        
        return yaw_oscillating;
    }

    static bool detect_single_motor_oscillation(const std::vector<double>& torque_history, double threshold)  {
        int sign_changes = 0;
        double max_amplitude = 0.0;
        double min_val = torque_history[0], max_val = torque_history[0];
        
        for (size_t i = 1; i < torque_history.size(); ++i) {
            if (torque_history[i] * torque_history[i-1] < 0) {
                sign_changes++;
            }
            min_val = std::min(min_val, torque_history[i]);
            max_val = std::max(max_val, torque_history[i]);
        }
        
        max_amplitude = max_val - min_val;
        
        return (sign_changes >= torque_history.size() / 2) &&
            (max_amplitude > threshold);
    }

    void detect_and_activate_deadzones() {
        bool gantry_oscillation = detect_gantry_oscillation();
        bool yaw_oscillation = detect_yaw_oscillation();
        
        if (gantry_oscillation && !gantry_dead_zone_active_) {
            activate_gantry_deadzone();
        }
        
        if (yaw_oscillation && !yaw_dead_zone_active_) {
            activate_yaw_deadzone();
        }
    }

    void compute_pid_control() {
    process_sensor_data();
    
    Eigen::Vector3d current_angles = {*left_motor_angle_, *right_motor_angle_, *yaw_motor_angle_};
    Eigen::Vector3d filtered_velocities = {*left_motor_filtered_velocity_, *right_motor_filtered_velocity_, *yaw_motor_filtered_velocity_};
    
    double sync_error = calculate_sync_error(current_angles.head<2>());

    // 计算前馈项
    double position_feedforward = calculate_position_feedforward(*gantry_control_angle_);
    double velocity_feedforward = calculate_velocity_feedforward(*gantry_control_angle_);

    // 左电机：双环PID + 前馈
    double left_angle_error = *gantry_control_angle_ - current_angles.x();
    double left_target_velocity = left_motor_velocity_pid_.update(left_angle_error) + velocity_feedforward;
    double left_torque = left_motor_torques_pid_.update(left_target_velocity - filtered_velocities.x()) 
                        + position_feedforward;
    
    // 右电机：双环PID + 前馈 + 同步补偿
    double right_angle_error = *gantry_control_angle_ - current_angles.y();
    double right_target_velocity = right_motor_velocity_pid_.update(right_angle_error) + velocity_feedforward;
    
    // 同步补偿在速度环加入
    double sync_compensation = sync_gain_ * sync_error;
    sync_compensation = std::clamp(sync_compensation, -max_sync_compensation_, max_sync_compensation_);
    right_target_velocity += sync_compensation;
    
    double right_torque = right_motor_torques_pid_.update(right_target_velocity - filtered_velocities.y()) 
                         + position_feedforward;
    
    // Yaw电机：双环PID + 前馈
    double yaw_angle_error = *yaw_control_angle_ - current_angles.z();
    double yaw_target_velocity = yaw_motor_velocity_pid_.update(yaw_angle_error);
    double yaw_torque = yaw_motor_torques_pid_.update(yaw_target_velocity - filtered_velocities.z());
    
    // 扭矩限幅
    left_torque = std::clamp(left_torque, -max_torque_, max_torque_);
    right_torque = std::clamp(right_torque, -max_torque_, max_torque_);
    yaw_torque = std::clamp(yaw_torque, -max_torque_, max_torque_);
    
    *left_motor_control_torque_ = left_torque;
    *right_motor_control_torque_ = right_torque;
    *yaw_motor_control_torque_ = yaw_torque;
    
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Target: %.3f, Torques: [%.3f, %.3f, %.3f], SyncErr: %.3f, FF: [%.3f, %.3f]",
                         *gantry_control_angle_,
                         *left_motor_control_torque_, *right_motor_control_torque_, *yaw_motor_control_torque_,
                         sync_error, position_feedforward, velocity_feedforward);
}

double calculate_position_feedforward(double target_angle) const {
    // 位置前馈：基于目标位置的静态补偿（如重力补偿）
    static double last_target = 0.0;
    double feedforward = 0.0;
    
    // 简单的位置前馈，可根据系统特性调整
    if (std::abs(target_angle - last_target) > 0.001) {
        feedforward = position_ff_gain_ * target_angle;
    }
    
    last_target = target_angle;
    return feedforward;
}

double calculate_velocity_feedforward(double target_angle) {
    // 速度前馈：基于目标位置的变化率
    static double last_target = 0.0;
    static auto last_time = now();
    
    auto current_time = now();
    double dt = (current_time - last_time).seconds();
    
    if (dt > 0.0) {
        double target_velocity = (target_angle - last_target) / dt;
        double feedforward = velocity_ff_gain_ * target_velocity;
        
        last_target = target_angle;
        last_time = current_time;
        
        return feedforward;
    }
    
    return 0.0;
}

    void reset_gantry_controls() {
        *left_motor_control_torque_  = 0.0;
        *right_motor_control_torque_ = 0.0;
    }
    void reset_yaw_controls() {
        *yaw_motor_control_torque_ = 0.0;
    }

    void reset_all_controls() {
        reset_gantry_controls();
        reset_yaw_controls();
    }

    void process_sensor_data() {
        *left_motor_filtered_velocity_  = left_velocity_filter_.update(*left_motor_velocity_);
        *right_motor_filtered_velocity_ = right_velocity_filter_.update(*right_motor_velocity_);
        *yaw_motor_filtered_velocity_   = yaw_velocity_filter_.update(*yaw_motor_velocity_);
    }

    static double calculate_sync_error(const Eigen::Vector2d& angles)  {
        return angles.x() - angles.y(); 
    }

    rclcpp::Logger logger_;
    filter::LowPassFilter<1> left_velocity_filter_;
    filter::LowPassFilter<1> right_velocity_filter_;
    filter::LowPassFilter<1> yaw_velocity_filter_;
    pid::PidCalculator left_motor_velocity_pid_, left_motor_torques_pid_;
    pid::PidCalculator right_motor_velocity_pid_, right_motor_torques_pid_;
    pid::PidCalculator yaw_motor_velocity_pid_, yaw_motor_torques_pid_;

    double gantry_oscillation_threshold_;
    double yaw_oscillation_threshold_;
    double gantry_dead_zone_duration_;
    double yaw_dead_zone_duration_;

    double sync_gain_;
    double position_ff_gain_, velocity_ff_gain_;
    double max_torque_, max_sync_compensation_;
    double load_compensation_gain_;
    int oscillation_window_;    
    
    bool gantry_dead_zone_active_ = false;
    bool yaw_dead_zone_active_ = false;
    
    std::vector<double> left_torque_history_;
    std::vector<double> right_torque_history_;
    std::vector<double> yaw_torque_history_;
    double last_control_angle_;
    double yaw_last_control_angle_;

    InputInterface<double> gantry_control_angle_;
    InputInterface<double>yaw_control_angle_;
    InputInterface<double> left_motor_angle_, left_motor_velocity_;
    InputInterface<double> right_motor_angle_, right_motor_velocity_;
    InputInterface<double> yaw_motor_angle_, yaw_motor_velocity_;

    OutputInterface<double> left_motor_filtered_velocity_;
    OutputInterface<double> right_motor_filtered_velocity_;
    OutputInterface<double> yaw_motor_filtered_velocity_;
    OutputInterface<double> left_motor_control_torque_;
    OutputInterface<double> right_motor_control_torque_;
    OutputInterface<double> yaw_motor_control_torque_;
};

}
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::GantryInformation, rmcs_executor::Component)