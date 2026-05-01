#pragma once

// ============================================================
// ArmController — motor control for 7-DOF arm over CAN
// ============================================================

#include <array>
#include <string>
#include <mutex>
#include "CanBridge.hpp"

// ============================================================
// MIT physical parameter ranges per motor model
// ============================================================

struct MotorParams {
    float p_min, p_max;    // position [rad]
    float v_min, v_max;    // velocity [rad/s]
    float t_min, t_max;    // torque [Nm]
    float kp_min, kp_max;
    float kd_min, kd_max;
    float kp_cmd, kd_cmd;  // default command gains
};

// CubeMars motor presets
//                        p_min   p_max   v_min   v_max   t_min   t_max  kp_min  kp_max  kd_min  kd_max  kp_cmd  kd_cmd
//                        [rad]   [rad]  [rad/s] [rad/s]   [Nm]    [Nm]    [-]     [-]     [-]     [-]     [-]     [-]
namespace MotorPresets {
    constexpr MotorParams AK45_36 = { -12.5f,  12.5f, -50.0f,  50.0f, -18.0f,  18.0f, 0.0f, 500.0f, 0.0f, 5.0f, 15.0f, 0.5f };
    constexpr MotorParams AK60_39 = { -12.5f,  12.5f, -10.0f,  10.0f, -80.0f,  80.0f, 0.0f, 500.0f, 0.0f, 5.0f,  20.0f, 0.5f };
    constexpr MotorParams AK45_10 = { -12.5f,  12.5f, -20.0f,  20.0f,  -8.0f,   8.0f, 0.0f, 500.0f, 0.0f, 5.0f, 15.0f, 0.5f };
    constexpr MotorParams AK40_10 = { -12.5f,  12.5f, -45.5f,  45.5f,  -5.0f,   5.0f, 0.0f, 500.0f, 0.0f, 5.0f, 15.0f, 0.5f };
}

// ============================================================
// Per-motor configuration — assign a preset to each motor ID
// ============================================================
// Index = motor_id - 1 (motor IDs 1..7)

static const std::array<MotorParams, 7> MOTOR_PARAMS = {{
    MotorPresets::AK45_36,  // motor 1
    MotorPresets::AK60_39,  // motor 2
    MotorPresets::AK45_36,  // motor 3
    MotorPresets::AK45_36,  // motor 4
    MotorPresets::AK45_10,  // motor 5
    MotorPresets::AK45_10,  // motor 6
    MotorPresets::AK40_10,  // motor 7 (gripper)
    
}};
// ============================================================

struct MITState {
    int     id          = 0;
    float   position    = 0.0f;  // [rad]
    float   velocity    = 0.0f;  // [rad/s]
    float   torque      = 0.0f;  // [Nm]
    uint8_t temperature = 0;     // [°C]
    uint8_t error       = 0;
    bool    valid       = false; // true after first real feedback
};

class ArmController {
public:
    static constexpr int NUM_MOTORS = 7;

    explicit ArmController(const std::string& can_port);
    ~ArmController();

    std::array<MITState, NUM_MOTORS> getMITStates() const {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return mit_states_;
    }

    void mitEnable(int motor_id);
    void mitDisable(int motor_id);
    void mitZero(int motor_id);
    void sendMIT(int motor_id, float p, float v, float kp, float kd, float torque);

    // Sends MIT command and waits for the response from that motor (max ~15 ms).
    // Returns true and updates internal state on success.
    bool sendMITAndReceive(int motor_id, float p, float v, float kp, float kd, float torque);

private:
    CanBridge can;
    mutable std::mutex can_mutex_;
    mutable std::mutex state_mutex_;
    std::array<MITState, NUM_MOTORS> mit_states_{};

    bool receiveMIT(MITState& mit);
};
