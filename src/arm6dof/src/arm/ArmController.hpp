#pragma once

#include <linux/can.h>
#include <string>
#include "CanBridge.hpp"

struct ServoState {
    int id;
    float position;
    float velocity;
    float torque;

};

class ArmController {
private:
    int motor_ids[6];
    CanBridge can;    
public:
    ArmController(const std::string& interface);
    bool ServoReceiveData(ServoState& state);
    bool setPosMotor(int motor_id, float degrees);
};

