#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>
#include <rmcs_msgs/keyboard.hpp>

namespace rmcs_core::controller {

enum class LauncherState : uint8_t {
    LOCKED    = 0,
    UNLOCKED  = 1,
    LOADING   = 2, 
    RESETTING = 3,
    FILLING   = 4,
    HOOKOFF   = 5,
    LAUNCHING = 6,
    ORIGEN    = 7
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
        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left",  joystick_left_);
        register_input("/remote/switch/right",   switch_right_);
        register_input("/remote/switch/left",    switch_left_);
        register_output("/launcher/state", launcher_state_);
        // register_input("/remote/keyboard",       keyboard_);          //maybe keyboard
        // register_input("/launcher/yaw/angle",    yaw_motor_angle_);
        // register_input("/launcher/launch/angle", launch_motor_angle_);
        // register_output("launcher/state/fire",   fire_);
        // register_output("/launcher/control/yaw_angle", yaw_control_angle_);
        // register_output("/launcher/control/launch_position", launch_control_position_);
        state_log_timer_ = create_wall_timer(
            std::chrono::seconds(2),
            [this]() {
                RCLCPP_INFO(get_logger(), "Current state: %s", to_string(state_).c_str());
            }
        );
    }

    void before_updating() override {
            if (!switch_left_.ready()) {
                RCLCPP_WARN(get_logger(), "Failed to fetch \"/switch_left\". Set to 0.0.");
            }
            if (!switch_right_.ready()) {
                RCLCPP_WARN(get_logger(), "Failed to fetch \"/switch_right\". Set to 0.0.");
            }
        }

    void update() override {
        using namespace rmcs_msgs;
        
        const auto switch_right = *switch_right_;
        const auto switch_left = *switch_left_;
        
        do {
            if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
                || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
                reset_all_controls();
                break;
            }                                
            
            auto mode = state_;
            if (switch_left != Switch::DOWN) {
                if (switch_left == Switch::MIDDLE && last_switch_right_ == Switch::DOWN && switch_right == Switch::MIDDLE) {
                    if (mode == LauncherState::LOCKED) {
                        mode =LauncherState::UNLOCKED;
                    } 
                } else if (switch_left == Switch::MIDDLE && last_switch_right_ == Switch::MIDDLE && switch_right == Switch::DOWN) {
                    if (mode == LauncherState::UNLOCKED) {
                        mode = LauncherState::LOCKED;
                    }
                } else if (switch_left == Switch::MIDDLE && last_switch_right_ == Switch::MIDDLE && switch_right == Switch::UP) {
                    if (mode == LauncherState::ORIGEN) {
                        mode = LauncherState::LOADING;
                    }
                } else if (switch_left == Switch::MIDDLE && last_switch_right_ == Switch::UP && switch_right == Switch::MIDDLE) {
                    if (mode == LauncherState::LOADING) {
                        mode = LauncherState::RESETTING;
                    }
                } else if (switch_left == Switch::UP && last_switch_right_ == Switch::MIDDLE && switch_right == Switch::DOWN) {
                    if (mode == LauncherState::RESETTING) {
                        mode = LauncherState::FILLING;
                    }
                } else if (switch_left == Switch::UP && last_switch_right_ == Switch::DOWN && switch_right == Switch::MIDDLE) {
                    if (mode == LauncherState::FILLING) {
                        mode = LauncherState::HOOKOFF;
                    }
                }else if (switch_left == Switch::UP && last_switch_right_ == Switch::MIDDLE && switch_right == Switch::UP) {
                    if (mode == LauncherState::HOOKOFF) {
                        mode = LauncherState::LAUNCHING;
                    }
                }else if (switch_left == Switch::UP && last_switch_right_ == Switch::UP && switch_right == Switch::MIDDLE) {
                    if (mode == LauncherState::LAUNCHING) {
                        mode = LauncherState::ORIGEN;
                    }
                }
            }
            state_ = mode;

            RCLCPP_INFO(this->get_logger(), "SwitchMode = %s", to_string(state_).c_str());
        } while(false);


        last_switch_right_ = switch_right;
        last_switch_left_ = switch_left;  
    }
    
private:

    void reset_all_controls() {
        //
    }

    static std::string to_string(LauncherState mode) {
        switch (mode) {
            case LauncherState::LOCKED:    return "LOCKED";
            case LauncherState::UNLOCKED:  return "UNLOCKED";
            case LauncherState::LOADING:   return "LOADING";
            case LauncherState::RESETTING: return "RESETTING";
            case LauncherState::FILLING:   return "FILLING";
            case LauncherState::HOOKOFF:   return "HOOKOFF";
            case LauncherState::LAUNCHING: return "LAUNCHING";
            case LauncherState::ORIGEN:    return "ORIGEN";
            default:                       return "UNKNOWN";
        }
    }

    static constexpr double inf = std::numeric_limits<double>::infinity();
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();
    
    rclcpp::Logger logger_;
    
    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;
    InputInterface<double> yaw_motor_angle_;
    InputInterface<double> launch_motor_angle_;

    OutputInterface<LauncherState> launcher_state_;
    
    // OutputInterface<double> gantry_control_angle_;
    // OutputInterface<double> yaw_control_angle_;
    // OutputInterface<double> launch_control_position_;
    
    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_switch_left_ = rmcs_msgs::Switch::UNKNOWN;
    LauncherState  state_ = LauncherState::LOCKED; 

    rclcpp::TimerBase::SharedPtr state_log_timer_;
};

} // namespace rmcs_core::controller

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::LauncherController, rmcs_executor::Component)
