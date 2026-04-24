#pragma once

#include <string>
#include "CanBridge.hpp"

// Stan odczytany z serwomechanizmu po ramce statusu 0x29xx.
struct ServoState {
    int   id;        // CAN ID silnika (1-6)
    float position;  // pozycja [stopnie * 10, raw z enkodera]
    float velocity;  // predkosc [eRPM raw]
    float torque;    // moment / prad [A]
    int8_t temp;     // temperatura MOS [°C]
};

// Sterowanie ramieniem 6-DOF przez adapter USB-CAN.
// Protokol CAN: ID = (typ_komendy << 8) | motor_id.
class ArmController {
public:
    explicit ArmController(const std::string& can_port);

    // Odbiera jedna ramke statusu z szyny CAN i wypelnia state.
    bool ServoReceiveData(ServoState& state);

    // Wyslij zadanie pozycji [stopnie] do silnika o podanym ID.
    bool setPosMotor(int motor_id, float degrees);

    // Wyslij zadanie pradu [A] do silnika o podanym ID.
    bool setCurrentMotor(int motor_id, float current);

private:
    static constexpr int NUM_MOTORS = 6;
    int motor_ids[NUM_MOTORS];
    CanBridge can;
};
