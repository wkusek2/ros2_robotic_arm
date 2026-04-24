#include <rclcpp/rclcpp.hpp>
#include <thread>
#include "ArmController.hpp"

class ArmNode : public rclcpp::Node {
    private:
        ArmController arm_controller;
        std::thread can_thread_;

        void canLoop(){
            while(rclcpp::ok()) {
                ServoState state;
                if(arm_controller.ServoReceiveData(state)) {
                    RCLCPP_INFO(this->get_logger(), "id:%d pos:%.1f vel:%.1f cur:%.2f temp:%d",
                        state.id, state.position, state.velocity, state.torque, state.temp);

                    

                }
            }
        }
    public:
        ArmNode() : Node("ArmNode"), arm_controller("/dev/ttyUSB0") {
            RCLCPP_INFO(this->get_logger(), "Arm Node Wystartowal");
            can_thread_ = std::thread(&ArmNode::canLoop, this);
        };

        ~ArmNode() {
            if(can_thread_.joinable()) can_thread_.join();
        }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmNode>());
    rclcpp::shutdown();
    return 0;
}