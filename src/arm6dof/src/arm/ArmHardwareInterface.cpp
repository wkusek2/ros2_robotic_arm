#include "ArmHardwareInterface.hpp"
#include "pluginlib/class_list_macros.hpp"



hardware_interface::CallbackReturn ArmHardwareInterface::on_init(const hardware_interface::HardwareInfo& info){
    (void)info;
    arm_controller_ = std::make_unique<ArmController>("/dev/ttyUSB0");
    hw_states_position_.resize(6, 0.0);
    hw_states_velocity_.resize(6, 0.0);
    hw_commands_.resize(6, 0.0);
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmHardwareInterface::on_activate(const rclcpp_lifecycle::State&) {
    for (int i = 1; i <= 6; i++)
        arm_controller_->mitEnable(i);
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmHardwareInterface::on_deactivate(const rclcpp_lifecycle::State&) {
    for (int i = 1; i <= 6; i++)
        arm_controller_->mitDisable(i);
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ArmHardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (int i = 0; i < 6; i++) {
        state_interfaces.emplace_back("joint" + std::to_string(i + 1), "position", &hw_states_position_[i]);
        state_interfaces.emplace_back("joint" + std::to_string(i + 1), "velocity", &hw_states_velocity_[i]);
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ArmHardwareInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (int i = 0; i < 6; i++) {
        command_interfaces.emplace_back("joint" + std::to_string(i + 1), "position", &hw_commands_[i]);
    }
    return command_interfaces;
}

hardware_interface::return_type ArmHardwareInterface::read(const rclcpp::Time&, const rclcpp::Duration&) {
    const auto states = arm_controller_->getMITStates();
    for (int i = 0; i < 6; i++) {
        hw_states_position_[i] = states[i].position;
        hw_states_velocity_[i] = states[i].velocity;
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArmHardwareInterface::write(const rclcpp::Time&, const rclcpp::Duration&) {
    for (int i = 0; i < 6; i++) {
        arm_controller_->sendMIT(i + 1, hw_commands_[i], 0.0f, 5.0f, 0.5f, 0.0f);
    }
    return hardware_interface::return_type::OK;
}

PLUGINLIB_EXPORT_CLASS(ArmHardwareInterface, hardware_interface::SystemInterface)


