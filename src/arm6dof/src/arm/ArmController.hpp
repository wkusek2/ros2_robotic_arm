#pragma once

#include <array>
#include <string>
#include "CanBridge.hpp"

// Stan odczytany z serwomechanizmu po ramce statusu 0x29xx.
struct ServoState {
    int    id;        // CAN ID silnika (1-6)
    float  position;  // pozycja [stopnie / 10.0 -> stopnie]
    float  velocity;  // predkosc [eRPM raw]
    float  torque;    // moment / prad [A]
    int8_t temp;      // temperatura MOS [°C]
};

// Sterowanie ramieniem 6-DOF przez adapter USB-CAN.
// Protokol CAN: ID = (typ_komendy << 8) | motor_id.
class ArmController {
public:
    static constexpr int NUM_MOTORS = 6;

    explicit ArmController(const std::string& can_port);

    // Odbiera jedna ramke statusu, aktualizuje states[] i wypelnia state.
    // Zwraca false przy timeoucie lub blednym ID.
    bool ServoReceiveData(ServoState& state);

    // Zwraca ostatni znany stan wszystkich silnikow (indeks = motor_id - 1).
    const std::array<ServoState, NUM_MOTORS>& getStates() const { return states_; }

    // Wyslij zadanie pozycji [stopnie] do silnika o podanym ID.
    bool setPosMotor(int motor_id, float degrees);

    // Wyslij zadanie pradu [A] do silnika o podanym ID.
    bool setCurrentMotor(int motor_id, float current);

    CanBridge& getCan() { return can; }

private:
    int motor_ids[NUM_MOTORS];
    CanBridge can;
    std::array<ServoState, NUM_MOTORS> states_{};  // bufor stanow wszystkich silnikow
};
