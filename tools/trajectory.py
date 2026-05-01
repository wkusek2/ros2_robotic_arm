#!/usr/bin/env python3

import serial
import time
import math
import threading
from collections import defaultdict

# =========================
# KONFIGURACJA
# =========================

CAN_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 2000000

DT = 1.0 / 35.0   # 50 Hz trajektorii

# Silnik 2 = CubeMars AK60-39, extended CAN MIT frame
AK_MOTOR_ID = 2

MOTOR_IDS = list(range(1, 8))  # 1..7

POS_A = 0.0       # [deg]
POS_B = 1.0    # [deg]

MAX_VEL = 30.0    # [deg/s]
ACCEL   = 10.0    # [deg/s^2]

# Stare silniki
KP     = 40.0
KD     = 1.0
TAU_FF = 0.0

# AK60-39 — delikatniejsze nastawy na start
AK_KP_CMD = 5.0
AK_KD_CMD = 0.1
AK_TAU_FF_CMD = 0.0

# Tryb diagnostyczny:
# po wysłaniu do danego silnika czekamy na jego feedback
WAIT_FOR_FEEDBACK = True
FEEDBACK_TIMEOUT = 0.005  # 5 ms

DEBUG_EVERY_SEC = 1.0

WRITE_WARN_MS = 5.0
LOOP_WARN_MS  = 2.0

# Stare MIT komendy dla silników 1,3,4,5,6,7
MIT_ENABLE  = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
MIT_DISABLE = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD])

# Stare zakresy MIT, np. T-Motor style
OLD_P_MIN, OLD_P_MAX   = -12.5, 12.5
OLD_V_MIN, OLD_V_MAX   = -50.0, 50.0
OLD_KP_MIN, OLD_KP_MAX =   0.0, 500.0
OLD_KD_MIN, OLD_KD_MAX =   0.0,   5.0
OLD_T_MIN, OLD_T_MAX   = -18.0,  18.0

# CubeMars AK60-39 MIT ranges
AK_P_MIN, AK_P_MAX     = -12.56, 12.56     # rad
AK_V_MIN, AK_V_MAX     = -10.0,  10.0      # rad/s
AK_KP_MIN, AK_KP_MAX   =   0.0, 500.0
AK_KD_MIN, AK_KD_MAX   =   0.0,   5.0
AK_T_MIN, AK_T_MAX     = -80.0,  80.0      # Nm

# CubeMars Control Mode IDs
AK_MODE_MIT     = 8
AK_MODE_DISABLE = 15

# Extended CAN ID dla AK60-39 ID=2:
# bits [28:8] = mode/function, bits [7:0] = driver ID
AK_MIT_CAN_ID     = (AK_MODE_MIT << 8) | AK_MOTOR_ID       # 0x802
AK_DISABLE_CAN_ID = (AK_MODE_DISABLE << 8) | AK_MOTOR_ID   # 0xF02


# =========================
# GLOBALNE LICZNIKI DEBUG
# =========================

feedback_counts = defaultdict(int)
feedback_total = defaultdict(int)

debug_lock = threading.Lock()
feedback_cond = threading.Condition(debug_lock)

debug_stats = {
    "rx_bytes": 0,
    "rx_frames": 0,
    "rx_std_frames": 0,
    "rx_ext_frames": 0,
    "rx_bad_header": 0,
    "rx_bad_tail": 0,
    "rx_bad_id": 0,
    "tx_frames": 0,
    "tx_bytes": 0,
    "tx_std_frames": 0,
    "tx_ext_frames": 0,
    "write_slow": 0,
    "write_timeout": 0,
    "feedback_timeout": 0,
    "loop_overrun": 0,
    "max_loop_ms": 0.0,
    "max_write_ms": 0.0,
}


# =========================
# UTILS
# =========================

def clamp(x, x_min, x_max):
    return max(x_min, min(x_max, x))


def float_to_uint(x, x_min, x_max, bits):
    x = clamp(x, x_min, x_max)
    span = x_max - x_min
    return int((x - x_min) * ((1 << bits) - 1) / span)


def int16_from_be(hi, lo):
    val = (hi << 8) | lo
    if val & 0x8000:
        val -= 0x10000
    return val


def get_feedback_seq(motor_id):
    with debug_lock:
        return feedback_total[motor_id]


def wait_for_feedback(motor_id, prev_seq, timeout):
    """
    Czeka aż feedback_total[motor_id] wzrośnie po wysłaniu komendy.
    Zwraca True jeśli przyszła odpowiedź, False jeśli timeout.
    """
    deadline = time.perf_counter() + timeout

    with feedback_cond:
        while feedback_total[motor_id] <= prev_seq:
            remaining = deadline - time.perf_counter()

            if remaining <= 0:
                return False

            feedback_cond.wait(timeout=remaining)

        return True


# =========================
# RAMKI ADAPTERA WAVESHARE / SERIAL-CAN
# =========================

def make_std_frame(can_id, payload):
    """
    Stary format:
    AA C8 ID_L ID_H DATA0..DATA7 55
    Razem 13 bajtów dla DLC=8.
    """
    if len(payload) != 8:
        raise ValueError("STD payload musi mieć 8 bajtów")

    return (
        bytes([
            0xAA,
            0xC8,
            can_id & 0xFF,
            (can_id >> 8) & 0x07,
        ])
        + payload
        + bytes([0x55])
    )


def make_ext_frame(can_id, payload):
    """
    Extended CAN frame:
    AA
    E0 | DLC
    29-bit ID little-endian: ID0 ID1 ID2 ID3
    DATA
    55
    """
    if len(payload) > 8:
        raise ValueError("EXT payload DLC > 8")

    if can_id > 0x1FFFFFFF:
        raise ValueError("Extended CAN ID > 29 bits")

    dlc = len(payload)

    return (
        bytes([
            0xAA,
            0xE0 | dlc,
            can_id & 0xFF,
            (can_id >> 8) & 0xFF,
            (can_id >> 16) & 0xFF,
            (can_id >> 24) & 0x1F,
        ])
        + payload
        + bytes([0x55])
    )


# =========================
# STARY MIT PAYLOAD DLA SILNIKÓW 1,3,4,5,6,7
# =========================

def make_old_mit_payload(p, v, kp, kd, torque):
    p_i  = float_to_uint(p,      OLD_P_MIN,  OLD_P_MAX,  16)
    v_i  = float_to_uint(v,      OLD_V_MIN,  OLD_V_MAX,  12)
    kp_i = float_to_uint(kp,     OLD_KP_MIN, OLD_KP_MAX, 12)
    kd_i = float_to_uint(kd,     OLD_KD_MIN, OLD_KD_MAX, 12)
    t_i  = float_to_uint(torque, OLD_T_MIN,  OLD_T_MAX,  12)

    return bytes([
        (p_i >> 8) & 0xFF,
         p_i       & 0xFF,
        (v_i >> 4) & 0xFF,
        ((v_i & 0x0F) << 4) | ((kp_i >> 8) & 0x0F),
         kp_i & 0xFF,
        (kd_i >> 4) & 0xFF,
        ((kd_i & 0x0F) << 4) | ((t_i >> 8) & 0x0F),
         t_i & 0xFF,
    ])


def make_old_mit_frame(motor_id, p, v, kp, kd, torque):
    payload = make_old_mit_payload(p, v, kp, kd, torque)
    return make_std_frame(motor_id, payload)


# =========================
# CUBEMARS AK60-39 MIT PAYLOAD
# =========================

def make_ak60_39_mit_payload(p, v, kp, kd, torque):
    """
    CubeMars AK60-39 MIT payload:
    DATA[0]      = KP high 8 bits
    DATA[1] 7-4  = KP low 4 bits
    DATA[1] 3-0  = KD high 4 bits
    DATA[2]      = KD low 8 bits
    DATA[3]      = Position high 8 bits
    DATA[4]      = Position low 8 bits
    DATA[5]      = Speed high 8 bits
    DATA[6] 7-4  = Speed low 4 bits
    DATA[6] 3-0  = Torque high 4 bits
    DATA[7]      = Torque low 8 bits
    """
    p_i  = float_to_uint(p,      AK_P_MIN,  AK_P_MAX,  16)
    v_i  = float_to_uint(v,      AK_V_MIN,  AK_V_MAX,  12)
    kp_i = float_to_uint(kp,     AK_KP_MIN, AK_KP_MAX, 12)
    kd_i = float_to_uint(kd,     AK_KD_MIN, AK_KD_MAX, 12)
    t_i  = float_to_uint(torque, AK_T_MIN,  AK_T_MAX,  12)

    return bytes([
        (kp_i >> 4) & 0xFF,
        ((kp_i & 0x0F) << 4) | ((kd_i >> 8) & 0x0F),
        kd_i & 0xFF,
        (p_i >> 8) & 0xFF,
        p_i & 0xFF,
        (v_i >> 4) & 0xFF,
        ((v_i & 0x0F) << 4) | ((t_i >> 8) & 0x0F),
        t_i & 0xFF,
    ])


def make_ak60_39_mit_frame(p, v, kp, kd, torque):
    payload = make_ak60_39_mit_payload(p, v, kp, kd, torque)
    return make_ext_frame(AK_MIT_CAN_ID, payload)


def make_ak60_39_disable_frame():
    """
    CubeMars disable:
    Extended ID = (15 << 8) | motor_id
    DLC = 0
    """
    return make_ext_frame(AK_DISABLE_CAN_ID, b"")


# =========================
# WYBÓR RAMKI DLA SILNIKA
# =========================

def make_motor_control_frame(motor_id, p, v, kp, kd, torque):
    if motor_id == AK_MOTOR_ID:
        return make_ak60_39_mit_frame(p, v, kp, kd, torque)

    return make_old_mit_frame(motor_id, p, v, kp, kd, torque)


def make_motor_enable_frame(motor_id):
    if motor_id == AK_MOTOR_ID:
        # AK60-39 w MIT mode nie używa starego FF FF ... FC.
        # Zaczyna działać po poprawnej ramce MIT extended.
        return None

    return make_std_frame(motor_id, MIT_ENABLE)


def make_motor_disable_frame(motor_id):
    if motor_id == AK_MOTOR_ID:
        return make_ak60_39_disable_frame()

    return make_std_frame(motor_id, MIT_DISABLE)


# =========================
# TRAJEKTORIA
# =========================

def trapezoidal(pos_start, pos_end, max_vel, accel, dt):
    dist = pos_end - pos_start
    direction = 1.0 if dist >= 0 else -1.0
    dist_abs = abs(dist)

    if dist_abs == 0:
        yield pos_end, 0.0
        return

    t_accel = max_vel / accel
    d_accel = 0.5 * accel * t_accel ** 2

    if 2 * d_accel >= dist_abs:
        t_accel = math.sqrt(dist_abs / accel)
        max_vel_real = accel * t_accel
        t_const = 0.0
        d_accel_real = 0.5 * accel * t_accel ** 2
    else:
        max_vel_real = max_vel
        t_const = (dist_abs - 2 * d_accel) / max_vel
        d_accel_real = d_accel

    t_total = 2 * t_accel + t_const
    t = 0.0

    while t <= t_total:
        if t < t_accel:
            vel = accel * t
            pos = 0.5 * accel * t ** 2

        elif t < t_accel + t_const:
            vel = max_vel_real
            pos = d_accel_real + max_vel_real * (t - t_accel)

        else:
            t_brake = t - t_accel - t_const
            vel = max_vel_real - accel * t_brake
            pos = (
                d_accel_real
                + max_vel_real * t_const
                + max_vel_real * t_brake
                - 0.5 * accel * t_brake ** 2
            )

        yield pos_start + direction * pos, direction * vel
        t += dt

    yield pos_end, 0.0


# =========================
# RX PARSER
# =========================

def register_feedback(motor_id):
    if 1 <= motor_id <= 7:
        feedback_counts[motor_id] += 1
        feedback_total[motor_id] += 1
        feedback_cond.notify_all()
    else:
        debug_stats["rx_bad_id"] += 1


def handle_std_rx_frame(frame):
    """
    AA C8 ID_L ID_H DATA0..DATA7 55
    """
    can_id = frame[2] | ((frame[3] & 0x07) << 8)
    motor_id = can_id

    with feedback_cond:
        debug_stats["rx_frames"] += 1
        debug_stats["rx_std_frames"] += 1
        register_feedback(motor_id)


def handle_ext_rx_frame(frame):
    """
    AA Ex ID0 ID1 ID2 ID3 DATA... 55

    CubeMars feedback:
    Extended CAN ID:
      bits [28:8] = Function ID
      bits [7:0]  = Drive ID

    Typowe feedback function ID:
      0x29 = realtime status
      0x2A = optional position frame
      0x2C = start frame
    """
    frame_type = frame[1]
    dlc = frame_type & 0x0F

    ext_id = (
        frame[2]
        | (frame[3] << 8)
        | (frame[4] << 16)
        | ((frame[5] & 0x1F) << 24)
    )

    func_id = (ext_id >> 8) & 0x1FFFFF
    drive_id = ext_id & 0xFF

    motor_id = drive_id

    with feedback_cond:
        debug_stats["rx_frames"] += 1
        debug_stats["rx_ext_frames"] += 1
        register_feedback(motor_id)

    # Debug danych AK60-39 real-time feedback 0x29.
    if drive_id == AK_MOTOR_ID and func_id == 0x29 and dlc == 8:
        data_start = 6
        data = frame[data_start:data_start + 8]

        pos_int = int16_from_be(data[0], data[1])
        spd_int = int16_from_be(data[2], data[3])
        cur_int = int16_from_be(data[4], data[5])
        temp = data[6]
        err = data[7]

        pos_deg = pos_int * 0.1
        erpm = spd_int * 10.0
        current_a = cur_int * 0.01

        with debug_lock:
            debug_stats["ak2_pos_deg"] = pos_deg
            debug_stats["ak2_erpm"] = erpm
            debug_stats["ak2_current_a"] = current_a
            debug_stats["ak2_temp"] = temp
            debug_stats["ak2_err"] = err


def receive_loop(ser, stop_event):
    buf = bytearray()

    while not stop_event.is_set():
        try:
            chunk = ser.read(512)
        except serial.SerialException as e:
            print(f"[RX ERROR] SerialException: {e}")
            break

        if not chunk:
            continue

        with debug_lock:
            debug_stats["rx_bytes"] += len(chunk)

        buf += chunk

        if len(buf) > 4096:
            with debug_lock:
                debug_stats["rx_bad_header"] += len(buf) - 128
            buf = buf[-128:]

        while len(buf) >= 2:
            if buf[0] != 0xAA:
                buf.pop(0)
                with debug_lock:
                    debug_stats["rx_bad_header"] += 1
                continue

            frame_type = buf[1]

            # Stary standard: AA C8 ID_L ID_H DATA0..DATA7 55
            if frame_type == 0xC8:
                frame_len = 13

                if len(buf) < frame_len:
                    break

                if buf[frame_len - 1] != 0x55:
                    buf.pop(0)
                    with debug_lock:
                        debug_stats["rx_bad_tail"] += 1
                    continue

                frame = bytes(buf[:frame_len])
                handle_std_rx_frame(frame)
                del buf[:frame_len]

            # Extended: AA Ex ID0 ID1 ID2 ID3 DATA... 55
            elif (frame_type & 0xF0) == 0xE0:
                dlc = frame_type & 0x0F
                frame_len = 6 + dlc + 1

                if len(buf) < frame_len:
                    break

                if buf[frame_len - 1] != 0x55:
                    buf.pop(0)
                    with debug_lock:
                        debug_stats["rx_bad_tail"] += 1
                    continue

                frame = bytes(buf[:frame_len])
                handle_ext_rx_frame(frame)
                del buf[:frame_len]

            else:
                buf.pop(0)
                with debug_lock:
                    debug_stats["rx_bad_header"] += 1


# =========================
# DEBUG PRINT
# =========================

def print_stats(stop_event):
    last_time = time.perf_counter()

    while not stop_event.is_set():
        time.sleep(DEBUG_EVERY_SEC)

        now = time.perf_counter()
        dt_real = now - last_time
        last_time = now

        with debug_lock:
            counts = dict(feedback_counts)
            for mid in MOTOR_IDS:
                feedback_counts[mid] = 0

            stats_snapshot = dict(debug_stats)

            # reset okresowy
            for key in [
                "rx_bytes",
                "rx_frames",
                "rx_std_frames",
                "rx_ext_frames",
                "rx_bad_header",
                "rx_bad_tail",
                "rx_bad_id",
                "tx_frames",
                "tx_bytes",
                "tx_std_frames",
                "tx_ext_frames",
                "write_slow",
                "write_timeout",
                "feedback_timeout",
                "loop_overrun",
            ]:
                debug_stats[key] = 0

            debug_stats["max_loop_ms"] = 0.0
            debug_stats["max_write_ms"] = 0.0

        rx_parts = [
            f"M{mid}:{counts.get(mid, 0) / dt_real:.0f}/s"
            for mid in MOTOR_IDS
        ]

        missing = [
            mid for mid in MOTOR_IDS
            if counts.get(mid, 0) == 0
        ]

        print(
            "\n[DEBUG]"
            f"\n  RX motors: {'  '.join(rx_parts)}"
            f"\n  RX frames: {stats_snapshot['rx_frames'] / dt_real:.0f}/s"
            f" | STD: {stats_snapshot['rx_std_frames'] / dt_real:.0f}/s"
            f" | EXT: {stats_snapshot['rx_ext_frames'] / dt_real:.0f}/s"
            f" | RX bytes: {stats_snapshot['rx_bytes'] / dt_real:.0f} B/s"
            f"\n  TX frames: {stats_snapshot['tx_frames'] / dt_real:.0f}/s"
            f" | STD: {stats_snapshot['tx_std_frames'] / dt_real:.0f}/s"
            f" | EXT: {stats_snapshot['tx_ext_frames'] / dt_real:.0f}/s"
            f" | TX bytes: {stats_snapshot['tx_bytes'] / dt_real:.0f} B/s"
            f"\n  Bad header: {stats_snapshot['rx_bad_header']}"
            f" | Bad tail: {stats_snapshot['rx_bad_tail']}"
            f" | Bad ID: {stats_snapshot['rx_bad_id']}"
            f"\n  Write slow: {stats_snapshot['write_slow']}"
            f" | Write timeout: {stats_snapshot['write_timeout']}"
            f" | Feedback timeout: {stats_snapshot['feedback_timeout']}"
            f" | Loop overrun: {stats_snapshot['loop_overrun']}"
            f"\n  Max write: {stats_snapshot['max_write_ms']:.3f} ms"
            f" | Max loop: {stats_snapshot['max_loop_ms']:.3f} ms"
        )

        if "ak2_pos_deg" in stats_snapshot:
            print(
                f"  AK60-39 M2 status:"
                f" pos={stats_snapshot.get('ak2_pos_deg', 0):.1f} deg"
                f" erpm={stats_snapshot.get('ak2_erpm', 0):.0f}"
                f" cur={stats_snapshot.get('ak2_current_a', 0):.2f} A"
                f" temp={stats_snapshot.get('ak2_temp', 0)} C"
                f" err={stats_snapshot.get('ak2_err', 0)}"
            )

        if missing:
            print(f"  [WARN] Brak feedbacku z silników: {missing}")


# =========================
# TX
# =========================

def classify_adapter_frame(frame):
    if len(frame) >= 2:
        if frame[1] == 0xC8:
            return "std"
        if (frame[1] & 0xF0) == 0xE0:
            return "ext"
    return "unknown"


def send_frame(ser, frame):
    t0 = time.perf_counter()

    try:
        n = ser.write(frame)

    except serial.SerialTimeoutException:
        with debug_lock:
            debug_stats["write_timeout"] += 1
        print("[TX WARN] write timeout")
        return False

    except serial.SerialException as e:
        print(f"[TX ERROR] SerialException: {e}")
        return False

    dt_write_ms = (time.perf_counter() - t0) * 1000.0
    kind = classify_adapter_frame(frame)

    with debug_lock:
        debug_stats["tx_frames"] += 1
        debug_stats["tx_bytes"] += n
        debug_stats["max_write_ms"] = max(debug_stats["max_write_ms"], dt_write_ms)

        if kind == "std":
            debug_stats["tx_std_frames"] += 1
        elif kind == "ext":
            debug_stats["tx_ext_frames"] += 1

        if dt_write_ms > WRITE_WARN_MS:
            debug_stats["write_slow"] += 1

    if dt_write_ms > WRITE_WARN_MS:
        print(f"[TX WARN] Wolne ser.write(): {dt_write_ms:.3f} ms")

    return True


def send_motor_and_wait_feedback(ser, mid, pos_rad, vel_rad, tau_ff):
    prev_seq = get_feedback_seq(mid)

    if mid == AK_MOTOR_ID:
        frame = make_motor_control_frame(
            mid,
            pos_rad,
            vel_rad,
            AK_KP_CMD,
            AK_KD_CMD,
            AK_TAU_FF_CMD,
        )
    else:
        frame = make_motor_control_frame(
            mid,
            pos_rad,
            vel_rad,
            KP,
            KD,
            tau_ff,
        )

    ok = send_frame(ser, frame)

    if not ok:
        print(f"[TX WARN] Nie wysłano ramki do silnika {mid}")
        return False

    if WAIT_FOR_FEEDBACK:
        got_fb = wait_for_feedback(mid, prev_seq, FEEDBACK_TIMEOUT)

        if not got_fb:
            with debug_lock:
                debug_stats["feedback_timeout"] += 1

            print(f"[FB WARN] Timeout feedbacku z M{mid}")
            return False

    return True


# =========================
# MAIN
# =========================

def main():
    print("Otwieram serial...")
    print(f"  Port: {CAN_PORT}")
    print(f"  Serial baud: {SERIAL_BAUD}")
    print(f"  DT traj: {DT:.4f} s = {1.0 / DT:.1f} Hz")
    print(f"  Motors: {MOTOR_IDS}")
    print(f"  AK60-39 motor ID: {AK_MOTOR_ID}")
    print(f"  AK MIT ExtID: 0x{AK_MIT_CAN_ID:X}")
    print(f"  AK Disable ExtID: 0x{AK_DISABLE_CAN_ID:X}")
    print(f"  KP={KP}, KD={KD}, TAU_FF={TAU_FF}")
    print(f"  AK_KP_CMD={AK_KP_CMD}, AK_KD_CMD={AK_KD_CMD}, AK_TAU_FF_CMD={AK_TAU_FF_CMD}")
    print(f"  POS: {POS_A} <-> {POS_B} deg")
    print(f"  MAX_VEL={MAX_VEL} deg/s, ACCEL={ACCEL} deg/s^2")
    print(f"  WAIT_FOR_FEEDBACK={WAIT_FOR_FEEDBACK}, FEEDBACK_TIMEOUT={FEEDBACK_TIMEOUT*1000:.1f} ms")

    ser = serial.Serial(
        CAN_PORT,
        SERIAL_BAUD,
        timeout=0.001,
        write_timeout=0.01,
    )

    ser.reset_input_buffer()
    ser.reset_output_buffer()

    stop_event = threading.Event()

    rx_thread = threading.Thread(
        target=receive_loop,
        args=(ser, stop_event),
        daemon=True,
    )

    stats_thread = threading.Thread(
        target=print_stats,
        args=(stop_event,),
        daemon=True,
    )

    rx_thread.start()
    stats_thread.start()

    print("\nMIT enable dla starych silników...")
    for mid in MOTOR_IDS:
        frame = make_motor_enable_frame(mid)

        if frame is None:
            print(f"  M{mid}: AK60-39 - pomijam stare MIT_ENABLE")
            continue

        send_frame(ser, frame)
        time.sleep(0.05)

    time.sleep(0.3)

    print("\nStart ruchu. Ctrl+C aby zatrzymać.\n")

    direction = 1
    gen = trapezoidal(POS_A, POS_B, MAX_VEL, ACCEL, DT)

    next_t = time.perf_counter()

    try:
        while True:
            loop_start = time.perf_counter()
            next_t += DT

            try:
                pos_deg, vel_deg = next(gen)

            except StopIteration:
                direction *= -1

                pos_s = POS_A if direction == 1 else POS_B
                pos_e = POS_B if direction == 1 else POS_A

                gen = trapezoidal(pos_s, pos_e, MAX_VEL, ACCEL, DT)
                pos_deg, vel_deg = next(gen)

                print(f"[INFO] Zmiana kierunku: {pos_s:.1f} -> {pos_e:.1f} deg")

            pos_rad = math.radians(pos_deg)
            vel_rad = math.radians(vel_deg)

            tau_ff = TAU_FF

            for mid in MOTOR_IDS:
                send_motor_and_wait_feedback(
                    ser,
                    mid,
                    pos_rad,
                    vel_rad,
                    tau_ff,
                )

            loop_ms = (time.perf_counter() - loop_start) * 1000.0

            with debug_lock:
                debug_stats["max_loop_ms"] = max(debug_stats["max_loop_ms"], loop_ms)

            sleep_t = next_t - time.perf_counter()

            if sleep_t > 0:
                time.sleep(sleep_t)

            else:
                overrun_ms = -sleep_t * 1000.0

                with debug_lock:
                    debug_stats["loop_overrun"] += 1

                if overrun_ms > LOOP_WARN_MS:
                    print(f"[LOOP WARN] Overrun: {overrun_ms:.3f} ms")

                next_t = time.perf_counter()

    except KeyboardInterrupt:
        print("\nCtrl+C")

    finally:
        print("\nZatrzymywanie...")
        stop_event.set()
        time.sleep(0.05)

        print(f"Disable -> {MOTOR_IDS}")
        for mid in MOTOR_IDS:
            frame = make_motor_disable_frame(mid)

            if frame is None:
                continue

            send_frame(ser, frame)
            time.sleep(0.02)

        ser.close()
        print("Serial zamknięty.")


if __name__ == "__main__":
    main()