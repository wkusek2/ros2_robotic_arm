#pragma once

#include "hardware_interface/system_interface.hpp"
#include "ArmController.hpp"
#include <vector>
#include <memory>

class ArmHardwareInterface : public hardware_interface::SystemInterface {
    public:
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    private:
    std::vector<double> hw_states_velocity_;
    std::vector<double> hw_states_position_;
    std::vector<double> hw_commands_;

    std::unique_ptr<ArmController> arm_controller_;


};