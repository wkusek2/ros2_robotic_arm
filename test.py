import serial
import time
import struct


COM_PORT = "COM6"
SERIAL_BAUD = 2_000_000

MOTOR_ID = 0x01      # CAN ID serwa
DT = 0.01            # 100 Hz na start, potem możesz zwiększyć


# Zakresy MIT - dopasuj do swojego modelu silnika
P_MIN = -12.5
P_MAX = 12.5

V_MIN = -50.0
V_MAX = 50.0

KP_MIN = 0.0
KP_MAX = 500.0

KD_MIN = 0.0
KD_MAX = 5.0

T_MIN = -18.0
T_MAX = 18.0


def float_to_uint(x, x_min, x_max, bits):
    """Konwersja float -> unsigned int dla protokołu MIT."""
    if x < x_min:
        x = x_min
    if x > x_max:
        x = x_max

    span = x_max - x_min
    return int((x - x_min) * ((1 << bits) - 1) / span)


def uint_to_float(x, x_min, x_max, bits):
    """Konwersja unsigned int -> float dla feedbacku MIT."""
    span = x_max - x_min
    return float(x) * span / float((1 << bits) - 1) + x_min


def waveshare_std_frame(can_id, data):
    """
    Opakowanie ramki CAN standard 11-bit dla Waveshare USB-CAN-A.
    
    Format:
    AA Cx ID_LOW ID_HIGH DATA... 55
    
    Dla DLC=8:
    AA C8 ID_LOW ID_HIGH DATA0..DATA7 55
    """
    if not 0 <= can_id <= 0x7FF:
        raise ValueError("Standard CAN ID musi być 0..0x7FF")

    if len(data) > 8:
        raise ValueError("CAN data max 8 bajtów")

    dlc = len(data)
    frame_type = 0xC0 | dlc  # standard frame, data frame, DLC

    id_low = can_id & 0xFF
    id_high = (can_id >> 8) & 0x07

    return bytes([0xAA, frame_type, id_low, id_high]) + bytes(data) + bytes([0x55])


def send_can(ser, can_id, data):
    packet = waveshare_std_frame(can_id, data)
    ser.write(packet)


def send_mit_enable(ser, motor_id):
    # 7x FF + FC
    data = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC]
    send_can(ser, motor_id, data)


def send_mit_disable(ser, motor_id):
    # 7x FF + FD
    data = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD]
    send_can(ser, motor_id, data)


def send_mit_zero(ser, motor_id):
    # 7x FF + FE
    data = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE]
    send_can(ser, motor_id, data)


def pack_mit_command(p, v, kp, kd, torque):
    """
    Pakowanie głównej ramki MIT:
    p      - pozycja [rad]
    v      - prędkość [rad/s]
    kp     - gain pozycji
    kd     - gain prędkości
    torque - moment feed-forward [Nm]
    """

    p_int = float_to_uint(p, P_MIN, P_MAX, 16)
    v_int = float_to_uint(v, V_MIN, V_MAX, 12)
    kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12)
    kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12)
    t_int = float_to_uint(torque, T_MIN, T_MAX, 12)

    data = [0] * 8

    data[0] = (p_int >> 8) & 0xFF
    data[1] = p_int & 0xFF

    data[2] = (v_int >> 4) & 0xFF
    data[3] = ((v_int & 0x0F) << 4) | ((kp_int >> 8) & 0x0F)

    data[4] = kp_int & 0xFF

    data[5] = (kd_int >> 4) & 0xFF
    data[6] = ((kd_int & 0x0F) << 4) | ((t_int >> 8) & 0x0F)

    data[7] = t_int & 0xFF

    return data


def send_mit_command(ser, motor_id, p, v, kp, kd, torque):
    data = pack_mit_command(p, v, kp, kd, torque)
    send_can(ser, motor_id, data)


def read_waveshare_frame(ser):
    """
    Odczyt jednej ramki Waveshare w formacie:
    AA Cx ID_LOW ID_HIGH DATA... 55

    Zwraca:
    can_id, data
    """
    while True:
        b = ser.read(1)
        if not b:
            return None

        if b[0] == 0xAA:
            break

    type_byte = ser.read(1)
    if len(type_byte) != 1:
        return None

    type_byte = type_byte[0]
    dlc = type_byte & 0x0F

    is_extended = bool(type_byte & 0x20)

    if is_extended:
        id_bytes = ser.read(4)
        if len(id_bytes) != 4:
            return None
        can_id = id_bytes[0] | (id_bytes[1] << 8) | (id_bytes[2] << 16) | (id_bytes[3] << 24)
    else:
        id_bytes = ser.read(2)
        if len(id_bytes) != 2:
            return None
        can_id = id_bytes[0] | (id_bytes[1] << 8)

    data = ser.read(dlc)
    if len(data) != dlc:
        return None

    tail = ser.read(1)
    if len(tail) != 1 or tail[0] != 0x55:
        return None

    return can_id, list(data)


def unpack_mit_feedback(data):
    """
    Feedback MIT:
    DATA[0] = motor ID
    DATA[1..2] = position
    DATA[3..4] = velocity
    DATA[4..5] = torque/current
    DATA[6] = temperature
    DATA[7] = error
    """
    if len(data) != 8:
        return None

    motor_id = data[0]

    p_int = (data[1] << 8) | data[2]
    v_int = (data[3] << 4) | (data[4] >> 4)
    t_int = ((data[4] & 0x0F) << 8) | data[5]

    temperature = data[6]
    error = data[7]

    p = uint_to_float(p_int, P_MIN, P_MAX, 16)
    v = uint_to_float(v_int, V_MIN, V_MAX, 12)
    torque = uint_to_float(t_int, T_MIN, T_MAX, 12)

    return {
        "id": motor_id,
        "position_rad": p,
        "velocity_rad_s": v,
        "torque_nm": torque,
        "temperature": temperature,
        "error": error,
    }


def main():
    ser = serial.Serial(
        port=COM_PORT,
        baudrate=SERIAL_BAUD,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.001,
    )

    print("Otwarty port:", COM_PORT)

    try:
        print("ENABLE MIT")
        send_mit_enable(ser, MOTOR_ID)
        time.sleep(0.2)

        # Bezpieczna komenda: trzymanie pozycji 0 rad
        p_des = 90.0
        v_des = 0.0
        kp = 20.0
        kd = 1.0
        torque_ff = 0.0

        while True:
            send_mit_command(
                ser,
                MOTOR_ID,
                p=p_des,
                v=v_des,
                kp=kp,
                kd=kd,
                torque=torque_ff,
            )

            frame = read_waveshare_frame(ser)
            if frame is not None:
                can_id, data = frame

                if len(data) == 8:
                    fb = unpack_mit_feedback(data)
                    if fb is not None:
                        print(
                            f"CAN_ID=0x{can_id:X} "
                            f"ID={fb['id']} "
                            f"pos={fb['position_rad']:.3f} rad "
                            f"vel={fb['velocity_rad_s']:.3f} rad/s "
                            f"torque={fb['torque_nm']:.3f} Nm "
                            f"temp={fb['temperature']} "
                            f"err={fb['error']}"
                        )

            time.sleep(DT)

    except KeyboardInterrupt:
        print("STOP")

    finally:
        print("DISABLE MIT")
        send_mit_disable(ser, MOTOR_ID)
        time.sleep(0.1)
        ser.close()


if __name__ == "__main__":
    main()