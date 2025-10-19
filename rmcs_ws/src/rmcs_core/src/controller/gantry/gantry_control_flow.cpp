#include <numbers>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller {

enum class SwitchMode : uint8_t {
    LOCKED       =0,
    UNLOCKED     =1,
};

enum class ControlMode : uint8_t {
    LOCKED         = 0,
    UNLOCKED       = 1,
    EMERGENCY_STOP = 2
};

class GantryController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    GantryController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger())
        {
        register_input("/remote/joystick/right",   joystick_right_);
        register_input("/remote/joystick/left",    joystick_left_);
        register_input("/remote/switch/right",     switch_right_);
        register_input("/remote/switch/left",      switch_left_);
        register_input("/gantry/left/angle", left_motor_angle_);
        register_output("/gantry/control/angle",             gantry_control_angle_);
        register_output("/gantry/control/mode",              gantry_control_mode_);
        register_output("/gantry/control/safe_operation",    safe_operation_);
    }

    void update() override {
        using namespace rmcs_msgs;

        auto switch_right = *switch_right_;
        auto switch_left  = *switch_left_;

        // 安全检查
        bool is_safe = is_safe_operation(switch_left, switch_right);
        *safe_operation_ = is_safe;

        if (!is_safe) {
            // 紧急停止模式
            *gantry_control_mode_ = static_cast<uint8_t>(ControlMode::EMERGENCY_STOP);
            *gantry_control_angle_ = 0.0; // 或者保持当前位置，根据安全策略
            return;
        }

        // 模式切换
        update_operation_mode(switch_left, switch_right);

        // 生成控制指令
        generate_control_commands();

        last_switch_right_ = switch_right;
        last_switch_left_ = switch_left;
    }

private:
    bool is_safe_operation(rmcs_msgs::Switch switch_left, rmcs_msgs::Switch switch_right) const {
        using namespace rmcs_msgs;
        if (switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, 
                               "Safety: Switch in unknown state");
            return false;
        }
        // 双拨杆下推为紧急停止
        if (switch_left == Switch::DOWN && switch_right == Switch::DOWN) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                               "Safety: Emergency stop activated");
            return false;
        }
        return true;
    }

    void update_operation_mode(rmcs_msgs::Switch switch_left, rmcs_msgs::Switch switch_right) {
        // 只有左拨杆不为DOWN时才允许模式切换
        if (switch_left != rmcs_msgs::Switch::DOWN) {
            handle_mode_transition(switch_right);
        }
        
        RCLCPP_DEBUG(get_logger(), "Current mode: %s", to_string(mode_).c_str());
    }

    void handle_mode_transition(rmcs_msgs::Switch switch_right) {
        using namespace rmcs_msgs;
        // 右拨杆从DOWN到MIDDLE：切换到UNLOCKED模式
        if (last_switch_right_ == Switch::DOWN && switch_right == Switch::MIDDLE) {
            if (mode_ == SwitchMode::LOCKED) {
                mode_ = SwitchMode::UNLOCKED;
                RCLCPP_INFO(get_logger(), "Mode transition: LOCKED -> UNLOCKED");
            }
        }
        // 右拨杆从MIDDLE到DOWN：切换到LOCKED模式
        else if (last_switch_right_ == Switch::MIDDLE && switch_right == Switch::DOWN) {
            if (mode_ == SwitchMode::UNLOCKED) {
                mode_ = SwitchMode::LOCKED;
                RCLCPP_INFO(get_logger(), "Mode transition: UNLOCKED -> LOCKED");
            }
        }
    }

    void generate_control_commands() {
        switch(mode_) {
            case SwitchMode::LOCKED:
                handle_locked_mode();
                break;
            case SwitchMode::UNLOCKED:
                handle_unlocked_mode();
                break;
        }
        // 限制控制角度范围（安全保护）
        limit_control_angle();

        // 设置输出控制模式
        *gantry_control_mode_ = static_cast<uint8_t>(mode_);
    }

    void handle_locked_mode() {
        using namespace rmcs_msgs;
        if (*switch_left_ == Switch::DOWN) {
            // 锁定模式：保持当前位置
            *gantry_control_angle_ = *left_motor_angle_;
        } else {
            // 非激活状态：归零
            *gantry_control_angle_ = 0.0;
        }
    }

    void handle_unlocked_mode() {
        if (!joystick_left_.ready()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                               "Joystick not ready in UNLOCKED mode");
            return;
        }
        
        // 解锁模式：根据摇杆输入调整控制角度
        double joystick_input = (*joystick_left_).x();
        double angle_increment = joystick_input_to_angle(joystick_input);
        *gantry_control_angle_ += angle_increment;
        
        RCLCPP_DEBUG(get_logger(), "Joystick input: %.3f -> Angle increment: %.3f", 
                    joystick_input, angle_increment);
    }

    void limit_control_angle() {
        constexpr double max_angle = M_PI; // 可根据实际机械限制调整
        *gantry_control_angle_ = std::clamp(*gantry_control_angle_, -max_angle, max_angle);
    }

    static double joystick_input_to_angle(double joystick_value) {
        constexpr double control_hz = 1000.0; // 控制频率
        constexpr double scale_factor = 2.0 * std::numbers::pi / control_hz;
        return joystick_value * scale_factor;
    }

    static std::string to_string(SwitchMode mode) {
        switch (mode) {
            case SwitchMode::LOCKED:   return "LOCKED";
            case SwitchMode::UNLOCKED: return "UNLOCKED";
            default:                   return "UNKNOWN";
        }
    }

private:
    static constexpr double inf = std::numeric_limits<double>::infinity();
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();
    rclcpp::Logger logger_;
    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<double> left_motor_angle_;
    OutputInterface<double> gantry_control_angle_;
    OutputInterface<uint8_t> gantry_control_mode_;
    OutputInterface<bool> safe_operation_;

    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_switch_left_  = rmcs_msgs::Switch::UNKNOWN;
    SwitchMode  mode_ = SwitchMode::LOCKED;     
};
}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::GantryController, rmcs_executor::Component)