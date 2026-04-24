#include "ArmController.hpp"

#include <iostream>
#include <vector>

// Typy komend CAN (gorny bajt ID).
static constexpr int CMD_SET_CURRENT = 1;  // payload: int32 [mA]
static constexpr int CMD_SET_POS     = 4;  // payload: int32 [stopnie * 1e6]

// ID statusu serwomechanizmow: gorny bajt = 0x29.
static constexpr uint32_t STATUS_ID_MASK  = 0xFF00;
static constexpr uint32_t STATUS_ID_VALUE = 0x2900;

ArmController::ArmController(const std::string& can_port) {
    for (int i = 0; i < NUM_MOTORS; i++)
        motor_ids[i] = i + 1;  // silniki 1..6

    can.open(can_port);
}

bool ArmController::ServoReceiveData(ServoState& state) {
    uint32_t id;
    std::vector<uint8_t> data;

    if (!can.receive(id, data)) return false;

    if ((id & STATUS_ID_MASK) != STATUS_ID_VALUE) return false;
    if (data.size() < 7) return false;

    state.id       = static_cast<int>(id & 0xFF);
    state.position = static_cast<int16_t>((data[0] << 8) | data[1]);
    state.velocity = static_cast<int16_t>((data[2] << 8) | data[3]);
    state.torque   = static_cast<int16_t>((data[4] << 8) | data[5]) / 10.0f;
    state.temp     = static_cast<uint8_t>(data[6]);

    return true;
}

bool ArmController::setPosMotor(int motor_id, float degrees) {
    int32_t raw = static_cast<int32_t>(degrees * 1000000.0f);
    std::vector<uint8_t> packet = {
        static_cast<uint8_t>((raw >> 24) & 0xFF),
        static_cast<uint8_t>((raw >> 16) & 0xFF),
        static_cast<uint8_t>((raw >>  8) & 0xFF),
        static_cast<uint8_t>((raw >>  0) & 0xFF),
    };
    can.send((CMD_SET_POS << 8) | motor_id, packet);
    return true;
}

bool ArmController::setCurrentMotor(int motor_id, float current) {
    int32_t ma = static_cast<int32_t>(current * 1000.0f);  // A -> mA
    std::vector<uint8_t> packet = {
        static_cast<uint8_t>((ma >> 24) & 0xFF),
        static_cast<uint8_t>((ma >> 16) & 0xFF),
        static_cast<uint8_t>((ma >>  8) & 0xFF),
        static_cast<uint8_t>((ma >>  0) & 0xFF),
    };
    can.send((CMD_SET_CURRENT << 8) | motor_id, packet);
    return true;
}
