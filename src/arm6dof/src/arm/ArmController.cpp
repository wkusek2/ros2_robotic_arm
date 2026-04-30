#include "ArmController.hpp"

#include <algorithm>
#include <iostream>
#include <vector>

// ============================================================
// CAN protocol constants — servo mode (VESC CAN)
// ============================================================
// Frame ID = (cmd << 8) | motor_id
// Payload: int32 big-endian

static constexpr int CMD_SET_CURRENT = 1;  // payload: current [mA]
static constexpr int CMD_SET_POS     = 4;  // payload: position [degrees * 1e6]

// Status ID mask: frames 0x2901..0x2907 are servo feedback.
static constexpr uint32_t STATUS_ID_MASK  = 0xFF00;
static constexpr uint32_t STATUS_ID_VALUE = 0x2900;

// ============================================================
// MIT protocol constants — physical ranges (AK45-36)
// ============================================================
// Float values are packed into uints of the given bit width,
// linearly mapped between [MIN, MAX]. Resolution:
//   position: 25 rad / 65535 = 0.00038 rad
//   velocity: 100 rad/s / 4095 = 0.024 rad/s
//   torque:   36 Nm / 4095 = 0.009 Nm

static constexpr float P_MIN  = -12.5f, P_MAX  =  12.5f;  // [rad]
static constexpr float V_MIN  = -50.0f, V_MAX  =  50.0f;  // [rad/s]
static constexpr float KP_MIN =   0.0f, KP_MAX = 500.0f;
static constexpr float KD_MIN =   0.0f, KD_MAX =   5.0f;
static constexpr float T_MIN  = -18.0f, T_MAX  =  18.0f;  // [Nm]

// ============================================================
// MIT conversion helpers
// ============================================================

// Maps a float from [xmin, xmax] to an unsigned int of 'bits' bits.
static uint32_t floatToUint(float x, float xmin, float xmax, int bits) {
    x = std::clamp(x, xmin, xmax);
    return static_cast<uint32_t>((x - xmin) / (xmax - xmin) * ((1 << bits) - 1));
}

// Maps an unsigned int of 'bits' bits back to a float in [xmin, xmax].
static float uintToFloat(uint32_t x, float xmin, float xmax, int bits) {
    return static_cast<float>(x) * (xmax - xmin) / static_cast<float>((1 << bits) - 1) + xmin;
}

// ============================================================
// Initialization
// ============================================================

ArmController::ArmController(const std::string& can_port) {
    for (int i = 0; i < NUM_MOTORS; i++)
        motor_ids[i] = i + 1;  // motors 1..6

    can.open(can_port);
    running_ = true;
    receive_thread_ = std::thread(&ArmController::receiveLoop, this);
}

ArmController::~ArmController() {
    running_ = false;
    if (receive_thread_.joinable())
        receive_thread_.join();
}

// ============================================================
// Receive loop — runs in a background thread
// ============================================================

void ArmController::receiveLoop() {
    ServoState servo;
    MITState   mit;
    while (running_) {
        if (!can.waitForData(100))
            continue;
        auto type = receiveAny(servo, mit);
        if (type == FrameType::MIT) {
            int idx = mit.id - 1;
            if (idx >= 0 && idx < NUM_MOTORS) {
                std::lock_guard<std::mutex> lock(state_mutex_);
                mit_states_[idx] = mit;
            }
        }
    }
}

// ============================================================
// Receive — receiveAny
// ============================================================
// Reads exactly one CAN frame and identifies its type:
//   SERVO — extended ID with upper byte 0x29, payload >=7B
//   MIT   — standard ID (<=0x7FF), payload ==8B with motor_id in data[0]
//   NONE  — timeout, sync error, or unknown type

FrameType ArmController::receiveAny(ServoState& servo, MITState& mit) {
    uint32_t id;
    std::vector<uint8_t> data;

    if (!can.receive(id, data)) return FrameType::NONE;

    // --- Servo frame ---
    if ((id & STATUS_ID_MASK) == STATUS_ID_VALUE && data.size() >= 7) {
        servo.id       = static_cast<int>(id & 0xFF);
        servo.position = static_cast<int16_t>((data[0] << 8) | data[1]) / 10.0f;
        servo.velocity = static_cast<int16_t>((data[2] << 8) | data[3]) / 10.0f;
        servo.torque   = static_cast<int16_t>((data[4] << 8) | data[5]) / 100.0f;
        servo.temp     = static_cast<uint8_t>(data[6]);

        int idx = servo.id - 1;
        if (idx >= 0 && idx < NUM_MOTORS)
            states_[idx] = servo;

        return FrameType::SERVO;
    }

    // --- MIT frame ---
    // Payload bit layout (8 bytes):
    //   [0..1]       position  16b
    //   [2] + [3>>4] velocity  12b
    //   [3&F] + [4]  kp        12b  (unused in feedback, same format)
    //   [5] + [6>>4] kd        12b
    //   [6&F] + [7]  torque    12b
    if (id <= 0x7FF && data.size() == 8 && data[0] >= 1 && data[0] <= NUM_MOTORS) {
        uint32_t p_i = (static_cast<uint32_t>(data[1]) << 8) | data[2];
        uint32_t v_i = (static_cast<uint32_t>(data[3]) << 4) | (data[4] >> 4);
        uint32_t t_i = ((static_cast<uint32_t>(data[4]) & 0x0F) << 8) | data[5];

        mit.id          = data[0];
        mit.position    = uintToFloat(p_i, P_MIN, P_MAX, 16);
        mit.velocity    = uintToFloat(v_i, V_MIN, V_MAX, 12);
        mit.torque      = uintToFloat(t_i, T_MIN, T_MAX, 12);
        mit.temperature = data[6];
        mit.error       = data[7];

        if (mit.error != 0)
            return FrameType::NONE;

        return FrameType::MIT;
    }

    return FrameType::NONE;
}

// ============================================================
// Transmit — servo mode
// ============================================================

bool ArmController::setPosMotor(int motor_id, float degrees) {
    int32_t raw = static_cast<int32_t>(degrees * 1000000.0f);
    std::vector<uint8_t> packet = {
        static_cast<uint8_t>((raw >> 24) & 0xFF),
        static_cast<uint8_t>((raw >> 16) & 0xFF),
        static_cast<uint8_t>((raw >>  8) & 0xFF),
        static_cast<uint8_t>((raw >>  0) & 0xFF),
    };
    std::lock_guard<std::mutex> lock(can_mutex_);
    can.send((CMD_SET_POS << 8) | motor_id, packet);
    return true;
}

bool ArmController::setCurrentMotor(int motor_id, float current) {
    int32_t ma = static_cast<int32_t>(current);
    std::vector<uint8_t> packet = {
        static_cast<uint8_t>((ma >> 24) & 0xFF),
        static_cast<uint8_t>((ma >> 16) & 0xFF),
        static_cast<uint8_t>((ma >>  8) & 0xFF),
        static_cast<uint8_t>((ma >>  0) & 0xFF),
    };
    std::lock_guard<std::mutex> lock(can_mutex_);
    can.send((CMD_SET_CURRENT << 8) | motor_id, packet);
    return true;
}

// ============================================================
// Transmit — MIT mode
// ============================================================
// Special commands: 7xFF + last byte distinguishes the action.
// MIT command frame: 8 bytes with packed float->uint fields.

static const std::vector<uint8_t> MIT_ENABLE  = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};
static const std::vector<uint8_t> MIT_DISABLE = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD};
static const std::vector<uint8_t> MIT_ZERO    = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE};

void ArmController::mitEnable(int motor_id) {
    std::lock_guard<std::mutex> lock(can_mutex_);
    can.sendStd(motor_id, MIT_ENABLE);
}

void ArmController::mitDisable(int motor_id) {
    std::lock_guard<std::mutex> lock(can_mutex_);
    can.sendStd(motor_id, MIT_DISABLE);
}

void ArmController::mitZero(int motor_id) {
    std::lock_guard<std::mutex> lock(can_mutex_);
    can.sendStd(motor_id, MIT_ZERO);
}

// Pack MIT command into 8 bytes:
//   data[0..1]  position  16b BE
//   data[2]     velocity  [11:4]
//   data[3]     velocity  [3:0] | kp [11:8]
//   data[4]     kp        [7:0]
//   data[5]     kd        [11:4]
//   data[6]     kd        [3:0] | torque [11:8]
//   data[7]     torque    [7:0]
void ArmController::sendMIT(int motor_id, float p, float v, float kp, float kd, float torque) {
    std::cerr << "MIT motor=" << motor_id << " p=" << p << " v=" << v << " kp=" << kp << " kd=" << kd << " tau=" << torque << "\n";

    uint32_t p_i  = floatToUint(p,      P_MIN,  P_MAX,  16);
    uint32_t v_i  = floatToUint(v,      V_MIN,  V_MAX,  12);
    uint32_t kp_i = floatToUint(kp,     KP_MIN, KP_MAX, 12);
    uint32_t kd_i = floatToUint(kd,     KD_MIN, KD_MAX, 12);
    uint32_t t_i  = floatToUint(torque, T_MIN,  T_MAX,  12);

    std::vector<uint8_t> data(8);
    data[0] =  (p_i >> 8) & 0xFF;
    data[1] =   p_i       & 0xFF;
    data[2] =  (v_i >> 4) & 0xFF;
    data[3] = ((v_i  & 0x0F) << 4) | ((kp_i >> 8) & 0x0F);
    data[4] =  kp_i & 0xFF;
    data[5] =  (kd_i >> 4) & 0xFF;
    data[6] = ((kd_i & 0x0F) << 4) | ((t_i  >> 8) & 0x0F);
    data[7] =   t_i & 0xFF;

    std::lock_guard<std::mutex> lock(can_mutex_);
    can.sendStd(motor_id, data);
}
