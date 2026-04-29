#pragma once

// ============================================================
// ArmController — motor control for 6-DOF arm over CAN
// ============================================================
// Intermediate layer between the ros2_control Hardware Interface
// and the USB-CAN adapter. Supports two communication modes
// for CubeMars AK45-36 motors:
//
//   Servo mode (VESC CAN):
//     - CMD_SET_POS    (cmd=4): position setpoint in degrees
//     - CMD_SET_CURRENT (cmd=1): current setpoint in mA
//     - Feedback: extended frame 0x29xx, ~150 Hz
//
//   MIT mode (CubeMars MIT Cheetah):
//     - 8B frame: [p(16b) | v(12b) | kp(12b) | kd(12b) | tau(12b)]
//     - Enable/disable/zero via special bytes 0xFC/0xFD/0xFE
//     - Feedback: standard frame, motor_id in data[0]
//
// Thread safety: can_mutex_ guards all CAN bus access.
// ============================================================

#include <array>
#include <string>
#include <mutex>
#include <thread>
#include <atomic>
#include "CanBridge.hpp"

// --- Feedback data structures ---

// Motor state in servo mode (frame 0x29xx).
struct ServoState {
    int    id;        // CAN motor ID (1-7)
    float  position;  // position [degrees], raw/10
    float  velocity;  // speed [eRPM/10]
    float  torque;    // phase current [A], raw/100
    int8_t temp;      // controller temperature [°C]
};

// Motor state in MIT mode (standard frame).
struct MITState {
    int     id;           // CAN motor ID (1-7)
    float   position;     // position [rad], range ±12.5
    float   velocity;     // velocity [rad/s], range ±50
    float   torque;       // torque [Nm], range ±18
    uint8_t temperature;  // temperature [°C]
    uint8_t error;        // error code (0 = none)
};

// Return type of receiveAny() — which frame was received.
enum class FrameType { NONE, SERVO, MIT };

// --- Controller class ---

class ArmController {
public:
    static constexpr int NUM_MOTORS = 6;

    explicit ArmController(const std::string& can_port);
    ~ArmController();

    // --- Receive ---

    // Reads one CAN frame and identifies its type (SERVO or MIT).
    // Populates the corresponding struct.
    FrameType receiveAny(ServoState& servo, MITState& mit);

    // Returns the buffer of last known MIT states (index = motor_id - 1).
    std::array<MITState, NUM_MOTORS> getMITStates() const {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return mit_states_;
    }

    // --- Transmit — servo mode ---

    // Position setpoint [degrees] via VESC CAN (CMD=4).
    bool setPosMotor(int motor_id, float degrees);

    // Current setpoint [A] via VESC CAN (CMD=1).
    bool setCurrentMotor(int motor_id, float current);

    // --- Transmit — MIT mode ---

    // Enables MIT mode on the motor (sends 7xFF + 0xFC).
    void mitEnable(int motor_id);

    // Disables MIT mode (sends 7xFF + 0xFD).
    void mitDisable(int motor_id);

    // Zeros the encoder at the current position (sends 7xFF + 0xFE).
    void mitZero(int motor_id);

    // Sends a MIT control command: position [rad], velocity [rad/s],
    // position gain kp, velocity gain kd, feed-forward torque [Nm].
    void sendMIT(int motor_id, float p, float v, float kp, float kd, float torque);

private:
    int    motor_ids[NUM_MOTORS];
    CanBridge can;
    mutable std::mutex can_mutex_;                      // guards CAN bus transmit
    mutable std::mutex state_mutex_;                    // guards mit_states_ buffer
    std::array<ServoState, NUM_MOTORS> states_{};
    std::array<MITState,   NUM_MOTORS> mit_states_{};   // last known MIT feedback per motor

    std::thread receive_thread_;
    std::atomic<bool> running_{false};
    void receiveLoop();
};
