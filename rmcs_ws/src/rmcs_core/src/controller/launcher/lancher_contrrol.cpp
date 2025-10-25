/*
#include <numbers>
#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller {

enum class ControlMode : uint8_t {
    MANUAL = 0,
    SEMI_AUTO = 1,
    AUTO = 2
};

enum class LaunchState : uint8_t {
    IDLE = 0,
    CHARGING = 1,
    READY = 2,
    FIRING = 3
};

class LauncherController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    LauncherController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {
        
        // 注册参数
        declare_parameter("max_yaw_angle", 3.14); // ±180度
        declare_parameter("max_launch_position", 10.0); // 最大发射位置
        declare_parameter("yaw_sensitivity", 0.01);
        declare_parameter("launch_sensitivity", 0.1);
        declare_parameter("charging_position", 5.0); // 充电位置
        declare_parameter("ready_position", 8.0);   // 准备发射位置
        
        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left",  joystick_left_);
        register_input("/remote/switch/right",   switch_right_);
        register_input("/remote/switch/left",    switch_left_);
        register_input("/remote/mouse/left",     mouse_left_);
        register_input("/remote/mouse/right",    mouse_right_);
        register_input("/launcher/yaw/angle",    yaw_motor_angle_);
        register_input("/launcher/launch/angle", launch_motor_angle_);
        
        register_output("/launcher/control/yaw_angle", yaw_control_angle_);
        register_output("/launcher/control/launch_position", launch_control_position_);
        register_output("/launcher/state/control_mode", control_mode_);
        register_output("/launcher/state/launch_state", launch_state_);
    }

    void before_updating() override {
        max_yaw_angle_ = get_parameter("max_yaw_angle").as_double();
        max_launch_position_ = get_parameter("max_launch_position").as_double();
        yaw_sensitivity_ = get_parameter("yaw_sensitivity").as_double();
        launch_sensitivity_ = get_parameter("launch_sensitivity").as_double();
        charging_position_ = get_parameter("charging_position").as_double();
        ready_position_ = get_parameter("ready_position").as_double();
    }

    void update() override {
        using namespace rmcs_msgs;
        
        auto switch_right = *switch_right_;
        auto switch_left = *switch_left_;
        
        // 模式切换逻辑
        update_control_mode(switch_right, switch_left);
        
        // 状态机更新
        update_launch_state_machine(switch_left, switch_right);
        
        // 根据模式生成控制指令
        switch(control_mode) {
            case ControlMode::MANUAL:
                update_manual_control();
                break;
            case ControlMode::SEMI_AUTO:
                update_semi_auto_control();
                break;
            case ControlMode::AUTO:
                update_auto_control();
                break;
        }
        
        // 限制控制输出范围
        *yaw_control_angle_ = std::clamp(*yaw_control_angle_, -max_yaw_angle_, max_yaw_angle_);
        *launch_control_position_ = std::clamp(*launch_control_position_, 0.0, max_launch_position_);
        
        last_switch_right_ = switch_right;
        last_switch_left_ = switch_left;
        
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 500,
                            "Mode: %d, State: %d, Yaw: %.3f, Launch: %.3f",
                            static_cast<int>(control_mode), static_cast<int>(launch_state),
                            *yaw_control_angle_, *launch_control_position_);
    }

private:
    void update_control_mode(rmcs_msgs::Switch switch_right, rmcs_msgs::Switch switch_left) {
        using namespace rmcs_msgs;
        
        // 右拨杆控制模式切换
        if (switch_right == Switch::UP) {
            control_mode = ControlMode::AUTO;
        } else if (switch_right == Switch::MIDDLE) {
            control_mode = ControlMode::SEMI_AUTO;
        } else if (switch_right == Switch::DOWN) {
            control_mode = ControlMode::MANUAL;
        }
        
        *control_mode_ = control_mode;
    }
    
    void update_launch_state_machine(rmcs_msgs::Switch switch_left, rmcs_msgs::Switch switch_right) {
        using namespace rmcs_msgs;
        
        switch(launch_state) {
            case LaunchState::IDLE:
                if (switch_left == Switch::UP) {
                    launch_state = LaunchState::CHARGING;
                    *launch_control_position_ = charging_position_;
                }
                break;
                
            case LaunchState::CHARGING:
                if (std::abs(*launch_motor_angle_ - charging_position_) < 0.1) {
                    launch_state = LaunchState::READY;
                    *launch_control_position_ = ready_position_;
                }
                if (switch_left == Switch::DOWN) {
                    launch_state = LaunchState::IDLE;
                    *launch_control_position_ = 0.0;
                }
                break;
                
            case LaunchState::READY:
                if (mouse_left_ && *mouse_left_ > 0.5) { // 鼠标左键发射
                    launch_state = LaunchState::FIRING;
                    *launch_control_position_ = max_launch_position_;
                }
                if (switch_left == Switch::DOWN) {
                    launch_state = LaunchState::IDLE;
                    *launch_control_position_ = 0.0;
                }
                break;
                
            case LaunchState::FIRING:
                // 发射完成后回到IDLE状态
                if (std::abs(*launch_motor_angle_ - max_launch_position_) < 0.1) {
                    launch_state_ = LaunchState::IDLE;
                    *launch_control_position_ = 0.0;
                }
                break;
        }
        
        *launch_state_ = launch_state;
    }
    
    void update_manual_control() {
        // 手动模式：左摇杆控制yaw，右摇杆控制发射位置
        if (joystick_left_.ready()) {
            double yaw_increment = (*joystick_left_).x() * yaw_sensitivity_;
            *yaw_control_angle_ += yaw_increment;
        }
        
        if (joystick_right_.ready()) {
            double launch_increment = (*joystick_right_).y() * launch_sensitivity_;
            *launch_control_position_ = std::clamp(*launch_control_position_ + launch_increment, 0.0, max_launch_position_);
        }
    }
    
    void update_semi_auto_control() {
        // 半自动模式：手动控制yaw，自动控制发射流程
        if (joystick_left_.ready()) {
            double yaw_increment = (*joystick_left_).x() * yaw_sensitivity_;
            *yaw_control_angle_ += yaw_increment;
        }
        // 发射位置由状态机控制
    }
    
    void update_auto_control() {
        // 自动模式：全部由状态机控制
        // yaw轴可以设置为跟踪目标等自动逻辑
        // 这里保持当前yaw角度
        if (std::isnan(*yaw_control_angle_)) {
            *yaw_control_angle_ = *yaw_motor_angle_;
        }
        // 发射位置由状态机控制
    }

private:
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();
    
    rclcpp::Logger logger_;
    
    // 参数
    double max_yaw_angle_;
    double max_launch_position_;
    double yaw_sensitivity_;
    double launch_sensitivity_;
    double charging_position_;
    double ready_position_;
    
    // 输入接口
    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<int> mouse_left_;
    InputInterface<int> mouse_right_;
    InputInterface<double> yaw_motor_angle_;
    InputInterface<double> launch_motor_angle_;
    
    // 输出接口
    OutputInterface<double> yaw_control_angle_;
    OutputInterface<double> launch_control_position_;
    OutputInterface<ControlMode> control_mode_;
    OutputInterface<LaunchState> launch_state_;
    
    // 内部状态
    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_switch_left_ = rmcs_msgs::Switch::UNKNOWN;
    ControlMode control_mode = ControlMode::MANUAL;
    LaunchState launch_state = LaunchState::IDLE;
};

} // namespace rmcs_core::controller

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::LauncherController, rmcs_executor::Component)
*/