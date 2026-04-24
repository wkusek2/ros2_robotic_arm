#include "ArmController.hpp"
#include <string>
#include <iostream>

ArmController::ArmController(const std::string& interface) {
    motor_ids[0] = 0x01;
    motor_ids[1] = 0x02;
    motor_ids[2] = 0x03;
    motor_ids[3] = 0x04;
    motor_ids[4] = 0x05;
    motor_ids[5] = 0x06;

    can.open(interface);

};

bool ArmController::ServoReceiveData(ServoState& state) {
    uint32_t id;
    std::vector<uint8_t> data;
    bool result = can.receive(id, data);
    if((id & 0xFF00) == 0x2900) {
        state.id = id & 0xFF;
        state.position = (int16_t)((data[0] << 8) | data[1]);
        state.torque = (int16_t)((data[4] << 8) | data[5]) / 10.0f;
        state.velocity = (int16_t)((data[2] << 8) | data[3]);
        state.temp = (uint8_t)(data[6]);


        

        
    }
    return result;
}

bool ArmController::setPosMotor(int motor_id, float degrees) {
    int cmd_type = 4;
    int32_t pos = (int32_t)(degrees * 1000000);
    std::vector<uint8_t> packet;
    packet.push_back(pos >> 24 & 0xFF);
    packet.push_back(pos >> 16 & 0xFF);
    packet.push_back(pos >> 8 & 0xFF);
    packet.push_back(pos >> 0 & 0xFF);
    can.send((cmd_type << 8) | motor_id, packet);
    return true;
}