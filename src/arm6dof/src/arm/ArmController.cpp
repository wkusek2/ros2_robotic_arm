#include "ArmController.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

static uint32_t floatToUint(float x, float xmin, float xmax, int bits) {
    x = std::clamp(x, xmin, xmax);
    return static_cast<uint32_t>((x - xmin) / (xmax - xmin) * ((1 << bits) - 1));
}

static float uintToFloat(uint32_t x, float xmin, float xmax, int bits) {
    return static_cast<float>(x) * (xmax - xmin) / static_cast<float>((1 << bits) - 1) + xmin;
}

// ============================================================
// Initialization
// ============================================================

ArmController::ArmController(const std::string& can_port) {
    can.open(can_port);
}

ArmController::~ArmController() {}

// ============================================================
// Receive
// ============================================================

bool ArmController::receiveMIT(MITState& mit) {
    uint32_t id;
    std::vector<uint8_t> data;

    if (!can.receive(id, data)) return false;
    if (data.size() != 8) return false;

    // AK60-39 extended feedback: CAN ID = motor_id | (0x29 << 8)
    // Layout: int16 pos (0.1 deg/LSB), int16 speed (10 ERPM/LSB), int16 current (0.01 A/LSB), temp, error
    if ((id >> 8) == 0x29) {
        uint8_t motor_id = static_cast<uint8_t>(id & 0xFF);
        if (motor_id < 1 || motor_id > NUM_MOTORS) return false;

        int16_t pos_i = static_cast<int16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]);
        int16_t cur_i = static_cast<int16_t>((static_cast<uint16_t>(data[4]) << 8) | data[5]);

        static constexpr float AK60_KT = 3.4616f;  // Nm/A
        mit.id          = motor_id;
        mit.position    = pos_i * (static_cast<float>(M_PI) / 1800.0f);  // 0.1 deg/LSB → rad
        mit.velocity    = 0.0f;  // ERPM→rad/s needs pole-pair count; left as 0
        mit.torque      = cur_i * 0.01f * AK60_KT;
        mit.temperature = data[6];
        mit.error       = data[7];
        mit.valid       = true;
        return true;
    }

    // Standard MIT feedback: data[0] = motor_id
    if (data[0] < 1 || data[0] > NUM_MOTORS) return false;

    uint32_t p_i = (static_cast<uint32_t>(data[1]) << 8) | data[2];
    uint32_t v_i = (static_cast<uint32_t>(data[3]) << 4) | (data[4] >> 4);
    uint32_t t_i = ((static_cast<uint32_t>(data[4]) & 0x0F) << 8) | data[5];

    const auto& mp  = MOTOR_PARAMS[data[0] - 1];
    mit.id          = data[0];
    mit.position    = uintToFloat(p_i, mp.p_min, mp.p_max, 16);
    mit.velocity    = uintToFloat(v_i, mp.v_min, mp.v_max, 12);
    mit.torque      = uintToFloat(t_i, mp.t_min, mp.t_max, 12);
    mit.temperature = data[6];
    mit.error       = data[7];
    mit.valid       = true;

    return true;
}

// ============================================================
// Transmit — MIT mode
// ============================================================

static const std::vector<uint8_t> MIT_ENABLE  = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};
static const std::vector<uint8_t> MIT_DISABLE = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD};
static const std::vector<uint8_t> MIT_ZERO    = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE};

void ArmController::mitEnable(int motor_id) {
    if (motor_id == 2) return;
    std::lock_guard<std::mutex> lock(can_mutex_);
    can.sendStd(motor_id, MIT_ENABLE);
}

void ArmController::mitDisable(int motor_id) {
    std::lock_guard<std::mutex> lock(can_mutex_);
    if (motor_id == 2) {
        // AK60-39 disable: extended frame, mode ID=15, DLC=0
        can.sendExt((15u << 8) | static_cast<uint32_t>(motor_id), {});
        return;
    }
    can.sendStd(motor_id, MIT_DISABLE);
}

void ArmController::mitZero(int motor_id) {
    std::lock_guard<std::mutex> lock(can_mutex_);
    can.sendStd(motor_id, MIT_ZERO);
}

// Pack MIT command into 8 bytes:
//   [0..1]  position  16b BE
//   [2]     velocity  [11:4]
//   [3]     velocity  [3:0] | kp [11:8]
//   [4]     kp        [7:0]
//   [5]     kd        [11:4]
//   [6]     kd        [3:0] | torque [11:8]
//   [7]     torque    [7:0]
void ArmController::sendMIT(int motor_id, float p, float v, float kp, float kd, float torque) {
    const auto& mp = MOTOR_PARAMS[motor_id - 1];

    uint32_t p_i  = floatToUint(p,      mp.p_min,  mp.p_max,  16);
    uint32_t v_i  = floatToUint(v,      mp.v_min,  mp.v_max,  12);
    uint32_t kp_i = floatToUint(kp,     mp.kp_min, mp.kp_max, 12);
    uint32_t kd_i = floatToUint(kd,     mp.kd_min, mp.kd_max, 12);
    uint32_t t_i  = floatToUint(torque, mp.t_min,  mp.t_max,  12);

    std::vector<uint8_t> data(8);

    std::lock_guard<std::mutex> lock(can_mutex_);

    if (motor_id == 2) {
        // CubeMars AK60-39 MIT packing:
        // order: KP, KD, Position, Velocity, Torque
        data[0] = (kp_i >> 4) & 0xFF;
        data[1] = ((kp_i & 0x0F) << 4) | ((kd_i >> 8) & 0x0F);
        data[2] = kd_i & 0xFF;

        data[3] = (p_i >> 8) & 0xFF;
        data[4] = p_i & 0xFF;

        data[5] = (v_i >> 4) & 0xFF;
        data[6] = ((v_i & 0x0F) << 4) | ((t_i >> 8) & 0x0F);
        data[7] = t_i & 0xFF;

        // MIT Control Mode ID = 8, extended frame
        uint32_t can_id = static_cast<uint32_t>(motor_id) | (8u << 8);
        can.sendExt(can_id, data);
        return;
    }

    // Existing MIT packing for the rest of motors:
    // order: Position, Velocity, KP, KD, Torque
    data[0] =  (p_i >> 8) & 0xFF;
    data[1] =   p_i       & 0xFF;
    data[2] =  (v_i >> 4) & 0xFF;
    data[3] = ((v_i  & 0x0F) << 4) | ((kp_i >> 8) & 0x0F);
    data[4] =  kp_i & 0xFF;
    data[5] =  (kd_i >> 4) & 0xFF;
    data[6] = ((kd_i & 0x0F) << 4) | ((t_i  >> 8) & 0x0F);
    data[7] =   t_i & 0xFF;

    can.sendStd(motor_id, data);
}

bool ArmController::sendMITAndReceive(int motor_id, float p, float v, float kp, float kd, float torque) {
    sendMIT(motor_id, p, v, kp, kd, torque);

    // Try up to 5 frames, 3 ms timeout each — skip frames from other motors
    for (int attempt = 0; attempt < 5; attempt++) {
        if (!can.waitForData(3)) break;
        MITState mit;
        if (!receiveMIT(mit)) continue;
        if (mit.id != motor_id) continue;
        std::lock_guard<std::mutex> lock(state_mutex_);
        mit_states_[motor_id - 1] = mit;
        return true;
    }
    return false;
}
