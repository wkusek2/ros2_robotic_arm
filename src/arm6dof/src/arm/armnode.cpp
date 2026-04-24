#include <rclcpp/rclcpp.hpp>
#include <thread>
#include "ArmController.hpp"

// Wezel ROS 2 sterujacy ramieniem 6-DOF przez CAN.
// Watki: ROS spin (glowny) + canLoop (odczyt statusu z szyny CAN).
class ArmNode : public rclcpp::Node {
public:
    ArmNode() : Node("ArmNode"), arm_controller("/dev/ttyUSB1") {
        RCLCPP_INFO(get_logger(), "ArmNode wystartowal");
        can_thread_ = std::thread(&ArmNode::canLoop, this);
    }

    ~ArmNode() {
        if (can_thread_.joinable()) can_thread_.join();
    }

private:
    ArmController arm_controller;
    std::thread   can_thread_;

    // Petla odczytu ramek statusu z szyny CAN.
    // Uruchamiana w osobnym watku — nie blokuje spinu ROS.
    void canLoop() {
        while (rclcpp::ok()) {
            ServoState state;
            if (arm_controller.ServoReceiveData(state)) {
                RCLCPP_INFO(get_logger(),
                    "id:%d  pos:%.1f  vel:%.1f  cur:%.2f A  temp:%d C",
                    state.id, state.position, state.velocity, state.torque, state.temp);
            }
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmNode>());
    rclcpp::shutdown();
    return 0;
}
