#include <rclcpp/rclcpp.hpp>
#include <thread>
#include "ArmController.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
// Wezel ROS 2 sterujacy ramieniem 6-DOF przez CAN.
// Watki: ROS spin (glowny) + canLoop (odczyt statusu z szyny CAN).
class ArmNode : public rclcpp::Node {
public:
    ArmNode() : Node("ArmNode"), arm_controller("/dev/ttyUSB1") {
        RCLCPP_INFO(get_logger(), "ArmNode wystartowal");
        joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("arm/diagnostics", 10);
        can_thread_ = std::thread(&ArmNode::canLoop, this);
    }

    ~ArmNode() {
        if (can_thread_.joinable()) can_thread_.join();
    }

private:
    ArmController arm_controller;
    std::thread   can_thread_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;

    // Petla odczytu ramek statusu z szyny CAN.
    // Uruchamiana w osobnym watku — nie blokuje spinu ROS.
    void canLoop() {
        while (rclcpp::ok()) {
                if (!arm_controller.getCan().waitForData(200))
                    continue;
                ServoState state;
                if (arm_controller.ServoReceiveData(state)) {
                RCLCPP_INFO(get_logger(),
                    "id:%d  pos:%.1f  vel:%.1f  cur:%.2f A  temp:%d C",
                    state.id, state.position, state.velocity, state.torque, state.temp);
                sensor_msgs::msg::JointState msg;
                msg.header.stamp = now();
                msg.name = { "joint_" + std::to_string(state.id) };
                msg.position = { state.position };
                msg.velocity = { state.velocity };
                msg.effort = { state.torque };
                joint_pub_->publish(msg);

                diagnostic_msgs::msg::DiagnosticArray diag_msg;
                diag_msg.header.stamp = now();

                diagnostic_msgs::msg::DiagnosticStatus status;
                status.name = "servo_" + std::to_string(state.id);
                status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;

                diagnostic_msgs::msg::KeyValue kv;
                kv.key = "temp_C";
                kv.value = std::to_string(state.temp);
                status.values.push_back(kv);
                diag_msg.status.push_back(status);
                diag_pub_->publish(diag_msg);
                
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
