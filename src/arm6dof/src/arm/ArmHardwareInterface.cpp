#include "ArmHardwareInterface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <cmath>



hardware_interface::CallbackReturn ArmHardwareInterface::on_init(const hardware_interface::HardwareInfo& info){
    (void)info;
    arm_controller_ = std::make_unique<ArmController>("/dev/ttyUSB0");
    hw_states_position_.resize(7, 0.0);
    hw_states_velocity_.resize(7, 0.0);
    hw_commands_.resize(7, 0.0);
    hw_commands_velocity_.resize(7, 0.0);

    node_ = std::make_shared<rclcpp::Node>("arm_hardware_interface");
    mit_enable_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
        "mit_enable", 10,
        [this](const std_msgs::msg::Int32::SharedPtr msg) {
            int id = msg->data;
            if (id == 0) {
                for (int i = 1; i <= 7; i++) arm_controller_->mitEnable(i);
            } else if (id >= 1 && id <= 7) {
                arm_controller_->mitEnable(id);
            }
        });
    executor_.add_node(node_);
    executor_thread_ = std::thread([this]() { executor_.spin(); });

    return hardware_interface::CallbackReturn::SUCCESS;
}

ArmHardwareInterface::~ArmHardwareInterface() {
    executor_.cancel();
    if (executor_thread_.joinable())
        executor_thread_.join();
}

hardware_interface::CallbackReturn ArmHardwareInterface::on_activate(const rclcpp_lifecycle::State&) {
    for (int i = 1; i <= 7; i++) {
        arm_controller_->mitEnable(i);
        sleep(2);
        arm_controller_->mitZero(i);
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmHardwareInterface::on_deactivate(const rclcpp_lifecycle::State&) {
    for (int i = 1; i <= 7; i++)
        arm_controller_->mitDisable(i);
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ArmHardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (int i = 0; i < 7; i++) {
        state_interfaces.emplace_back("joint" + std::to_string(i + 1), "position", &hw_states_position_[i]);
        state_interfaces.emplace_back("joint" + std::to_string(i + 1), "velocity", &hw_states_velocity_[i]);
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ArmHardwareInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (int i = 0; i < 7; i++) {
        command_interfaces.emplace_back("joint" + std::to_string(i + 1), "position", &hw_commands_[i]);
        command_interfaces.emplace_back("joint" + std::to_string(i + 1), "velocity", &hw_commands_velocity_[i]);
    }
    return command_interfaces;
}

hardware_interface::return_type ArmHardwareInterface::read(const rclcpp::Time&, const rclcpp::Duration&) {
    static constexpr double MAX_POS_JUMP = 0.5;
    static std::array<bool, 7> seeded{};
    const auto states = arm_controller_->getMITStates();
    for (int i = 0; i < 7; i++) {
        if (!states[i].valid) continue;
        double new_pos = states[i].position;
        double new_vel = states[i].velocity;
        if (std::abs(new_pos) <= 12.5) {
            if (!seeded[i] || std::abs(new_pos - hw_states_position_[i]) <= MAX_POS_JUMP) {
                hw_states_position_[i] = new_pos;
                seeded[i] = true;
            }
        }
        if (std::abs(new_vel) <= 50.0)
            hw_states_velocity_[i] = new_vel;
    }
    return hardware_interface::return_type::OK;
}

// Only send to connected motors (1 and 3); expand when new motors arrive.
static const int ACTIVE_MOTORS[] = {1, 2, 3, 4, 5, 6, 7};

static std::array<bool, 7> cmd_seeded{};

hardware_interface::return_type ArmHardwareInterface::write(const rclcpp::Time&, const rclcpp::Duration&) {
    for (int motor_id : ACTIVE_MOTORS) {
        int i = motor_id - 1;
        if (!cmd_seeded[i]) {
            if (std::abs(hw_states_position_[i]) <= 12.5) {
                hw_commands_[i] = hw_states_position_[i];
                cmd_seeded[i] = true;
            } else {
                continue;  // czekaj na pierwszy feedback zanim zaczniesz wysylac
            }
        }
        const auto& mp = MOTOR_PARAMS[i];
        arm_controller_->sendMITAndReceive(motor_id, static_cast<float>(hw_commands_[i]),
                                          static_cast<float>(hw_commands_velocity_[i]), mp.kp_cmd, mp.kd_cmd, 0.0f);
    }
    return hardware_interface::return_type::OK;
}

PLUGINLIB_EXPORT_CLASS(ArmHardwareInterface, hardware_interface::SystemInterface)


