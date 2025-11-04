#include <memory>

#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rclcpp/logger.hpp>
#include <rmcs_executor/component.hpp>
#include <std_msgs/msg/int32.hpp>

#include <librmcs/client/cboard.hpp>

#include "hardware/device/bmi088.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"

namespace rmcs_core::hardware {

class Launcher
    : public rmcs_executor::Component
    , public rclcpp::Node
    , private librmcs::client::CBoard {
public:
    Launcher()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , librmcs::client::CBoard{static_cast<int>(get_parameter("usb_pid").as_int())}
        , logger_(get_logger())
        , launcher_command_(
              create_partner_component<LauncherCommand>(get_component_name() + "_command", *this))
        , pulling_motors_(
            {*this, *launcher_command_, "/launcher/pulling_motor/left"},
            {*this, *launcher_command_, "/launcher/pulling_motor/right"})
        , pitch_motors_(
            {*this, *launcher_command_, "/launcher/first/pitch"},
            {*this, *launcher_command_, "/launcher/second/pitch"})
        , position_setting_motor_(*this, *launcher_command_, "/launcher/position_setting/motor")
        , yaw_setting_motor_(*this, *launcher_command_, "/launcher/yaw")
        , dr16_{*this}
        , bmi088_(1000, 0.2, 0.0)
        , transmit_buffer_(*this, 32)
        , event_thread_([this]() { handle_events(); }) {
        
        for (auto& motor : pulling_motors_)
            motor.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::M3508}.set_encoder_zero_point(
                static_cast<int>(get_parameter("pulling_motors_zero_point").as_int())));
        for (auto& motor : pitch_motors_)
            motor.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::M3508}.set_encoder_zero_point(
                static_cast<int>(get_parameter("pitch_motors_zero_point").as_int())));
        position_setting_motor_.configure(device::DjiMotor::Config{device::DjiMotor::Type::GM6020}.set_encoder_zero_point(
                static_cast<int>(get_parameter("position_setting_motor_zero_point").as_int())));
        yaw_setting_motor_.configure(device::DjiMotor::Config{device::DjiMotor::Type::GM6020}.set_encoder_zero_point(
                static_cast<int>(get_parameter("yaw_setting_motor_zero_point").as_int())));
    }

    ~Launcher() override {
        stop_handling_events();
        if (event_thread_.joinable()) {
            event_thread_.join();
    }
}

    void update() override { 
        update_motors();
        dr16_.update_status();
    }

    void command_update() { 
        uint16_t can_commands[4];

        can_commands[0] = position_setting_motor_.generate_command();
        can_commands[1] = yaw_setting_motor_.generate_command();
        can_commands[2] = 0;
        can_commands[3] = 0;
        transmit_buffer_.add_can1_transmission(0x1FE, std::bit_cast<uint64_t>(can_commands));

        can_commands[0] = pulling_motors_[0].generate_command();
        can_commands[1] = pulling_motors_[1].generate_command();
        can_commands[2] = pitch_motors_[0].generate_command();
        can_commands[3] = pitch_motors_[1].generate_command();
        transmit_buffer_.add_can1_transmission(0x200, std::bit_cast<uint64_t>(can_commands));

        can_commands[0] = 0;
        can_commands[1] = 0;
        can_commands[2] = 0;
        can_commands[3] = 0;
        transmit_buffer_.add_can2_transmission(0x1FE, std::bit_cast<uint64_t>(can_commands));
        transmit_buffer_.add_can2_transmission(0x200, std::bit_cast<uint64_t>(can_commands));

        transmit_buffer_.trigger_transmission();
    }
private:
    void update_motors() {
        using namespace rmcs_description;
        for (auto& motor : pulling_motors_)
            motor.update_status();

        for (auto& motor : pitch_motors_)
            motor.update_status();

        position_setting_motor_.update_status();
        yaw_setting_motor_.update_status();
    }


    class LauncherCommand : public rmcs_executor::Component {
    public:
        explicit LauncherCommand(Launcher& launcher)
            : launcher_(launcher) {}

        void update() override { 
            launcher_.command_update(); 
        }

        Launcher& launcher_;
    };

protected:
    void can1_receive_callback(
        uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission, 
        uint8_t can_data_length) override {
        if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
            return;

        if (can_id == 0x201) {
            auto& motor = pulling_motors_[0];
            motor.store_status(can_data);
        } else if (can_id == 0x202) {
            auto& motor = pulling_motors_[1];
            motor.store_status(can_data);
        } else if (can_id == 0x203) {
            auto& motor = pitch_motors_[0];
            motor.store_status(can_data);
        } else if (can_id == 0x204) {
            auto& motor = pitch_motors_[1];
            motor.store_status(can_data);
        } else if (can_id == 0x205) {
            position_setting_motor_.store_status(can_data);
        } else if (can_id == 0x206) {
            yaw_setting_motor_.store_status(can_data);
        }
    }

private:

    // void uart1_receive_callback(const std::byte*, uint8_t) override {}
    // void uart2_receive_callback(const std::byte*, uint8_t) override {}
    // void dbus_receive_callback(const std::byte*, uint8_t) override {}
    // void accelerometer_receive_callback(int16_t, int16_t, int16_t) override {}
    // void gyroscope_receive_callback(int16_t, int16_t, int16_t) override {}

    rclcpp::Logger logger_;

    std::shared_ptr<LauncherCommand> launcher_command_;
    
    device::DjiMotor pulling_motors_[2];
    device::DjiMotor pitch_motors_[2];
    device::DjiMotor position_setting_motor_;
    device::DjiMotor yaw_setting_motor_;

    rmcs_core::hardware::device::Dr16 dr16_;
    device::Bmi088 bmi088_;
    librmcs::client::CBoard::TransmitBuffer transmit_buffer_;

    std::thread event_thread_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Launcher, rmcs_executor::Component)