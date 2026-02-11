import serial
import time
import numpy as np
from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
import os

crc16_tab = [
    0x0000,
    0x1021,
    0x2042,
    0x3063,
    0x4084,
    0x50A5,
    0x60C6,
    0x70E7,
    0x8108,
    0x9129,
    0xA14A,
    0xB16B,
    0xC18C,
    0xD1AD,
    0xE1CE,
    0xF1EF,
    0x1231,
    0x0210,
    0x3273,
    0x2252,
    0x52B5,
    0x4294,
    0x72F7,
    0x62D6,
    0x9339,
    0x8318,
    0xB37B,
    0xA35A,
    0xD3BD,
    0xC39C,
    0xF3FF,
    0xE3DE,
    0x2462,
    0x3443,
    0x0420,
    0x1401,
    0x64E6,
    0x74C7,
    0x44A4,
    0x5485,
    0xA56A,
    0xB54B,
    0x8528,
    0x9509,
    0xE5EE,
    0xF5CF,
    0xC5AC,
    0xD58D,
    0x3653,
    0x2672,
    0x1611,
    0x0630,
    0x76D7,
    0x66F6,
    0x5695,
    0x46B4,
    0xB75B,
    0xA77A,
    0x9719,
    0x8738,
    0xF7DF,
    0xE7FE,
    0xD79D,
    0xC7BC,
    0x48C4,
    0x58E5,
    0x6886,
    0x78A7,
    0x0840,
    0x1861,
    0x2802,
    0x3823,
    0xC9CC,
    0xD9ED,
    0xE98E,
    0xF9AF,
    0x8948,
    0x9969,
    0xA90A,
    0xB92B,
    0x5AF5,
    0x4AD4,
    0x7AB7,
    0x6A96,
    0x1A71,
    0x0A50,
    0x3A33,
    0x2A12,
    0xDBFD,
    0xCBDC,
    0xFBBF,
    0xEB9E,
    0x9B79,
    0x8B58,
    0xBB3B,
    0xAB1A,
    0x6CA6,
    0x7C87,
    0x4CE4,
    0x5CC5,
    0x2C22,
    0x3C03,
    0x0C60,
    0x1C41,
    0xEDAE,
    0xFD8F,
    0xCDEC,
    0xDDCD,
    0xAD2A,
    0xBD0B,
    0x8D68,
    0x9D49,
    0x7E97,
    0x6EB6,
    0x5ED5,
    0x4EF4,
    0x3E13,
    0x2E32,
    0x1E51,
    0x0E70,
    0xFF9F,
    0xEFBE,
    0xDFDD,
    0xCFFC,
    0xBF1B,
    0xAF3A,
    0x9F59,
    0x8F78,
    0x9188,
    0x81A9,
    0xB1CA,
    0xA1EB,
    0xD10C,
    0xC12D,
    0xF14E,
    0xE16F,
    0x1080,
    0x00A1,
    0x30C2,
    0x20E3,
    0x5004,
    0x4025,
    0x7046,
    0x6067,
    0x83B9,
    0x9398,
    0xA3FB,
    0xB3DA,
    0xC33D,
    0xD31C,
    0xE37F,
    0xF35E,
    0x02B1,
    0x1290,
    0x22F3,
    0x32D2,
    0x4235,
    0x5214,
    0x6277,
    0x7256,
    0xB5EA,
    0xA5CB,
    0x95A8,
    0x8589,
    0xF56E,
    0xE54F,
    0xD52C,
    0xC50D,
    0x34E2,
    0x24C3,
    0x14A0,
    0x0481,
    0x7466,
    0x6447,
    0x5424,
    0x4405,
    0xA7DB,
    0xB7FA,
    0x8799,
    0x97B8,
    0xE75F,
    0xF77E,
    0xC71D,
    0xD73C,
    0x26D3,
    0x36F2,
    0x0691,
    0x16B0,
    0x6657,
    0x7676,
    0x4615,
    0x5634,
    0xD94C,
    0xC96D,
    0xF90E,
    0xE92F,
    0x99C8,
    0x89E9,
    0xB98A,
    0xA9AB,
    0x5844,
    0x4865,
    0x7806,
    0x6827,
    0x18C0,
    0x08E1,
    0x3882,
    0x28A3,
    0xCB7D,
    0xDB5C,
    0xEB3F,
    0xFB1E,
    0x8BF9,
    0x9BD8,
    0xABBB,
    0xBB9A,
    0x4A75,
    0x5A54,
    0x6A37,
    0x7A16,
    0x0AF1,
    0x1AD0,
    0x2AB3,
    0x3A92,
    0xFD2E,
    0xED0F,
    0xDD6C,
    0xCD4D,
    0xBDAA,
    0xAD8B,
    0x9DE8,
    0x8DC9,
    0x7C26,
    0x6C07,
    0x5C64,
    0x4C45,
    0x3CA2,
    0x2C83,
    0x1CE0,
    0x0CC1,
    0xEF1F,
    0xFF3E,
    0xCF5D,
    0xDF7C,
    0xAF9B,
    0xBFBA,
    0x8FD9,
    0x9FF8,
    0x6E17,
    0x7E36,
    0x4E55,
    0x5E74,
    0x2E93,
    0x3EB2,
    0x0ED1,
    0x1EF0,
]

Servo_Params_Serial = {
    "ERROR_CODES": {
        0: "FAULT_CODE_NONE",
        1: "FAULT_CODE_OVER_VOLTAGE",
        2: "FAULT_CODE_UNDER_VOLTAGE",
        3: "FAULT_CODE_DRIVE",
        4: "FAULT_CODE_ABS_OVER_CURRENT",
        5: "FAULT_CODE_OVER_TEMP_FET",
        6: "FAULT_CODE_OVER_TEMP_MOTOR",
        7: "FAULT_CODE_GATE_DRIVER_OVER_VOLTAGE",
        8: "FAULT_CODE_GATE_DRIVER_UNDER_VOLTAGE",
        9: "FAULT_CODE_MCU_UNDER_VOLTAGE",
        10: "FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET",
        11: "FAULT_CODE_ENCODER_SPI",
        12: "FAULT_CODE_ENCODER_SINCOS_BELOW_MIN_AMPLITUDE",
        13: "FAULT_CODE_ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE",
        14: "FAULT_CODE_FLASH_CORRUPTION",
        15: "FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1",
        16: "FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_2",
        17: "FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_3",
        18: "FAULT_CODE_UNBALANCED_CURRENTS",
    },
    "PARAMETER_FLAGS": {
        # parameter       : flag
        "MOSFET_TEMP": 1,
        "MOTOR_TEMP": 1 << 2,
        "OUTPUT_CURRENT": 1 << 3,
        "INPUT_CURRENT": 1 << 4,
        "D_CURRENT": 1 << 5,
        "Q_CURRENT": 1 << 6,
        "DUTY_CYCLE": 1 << 7,
        "MOTOR_SPEED": 1 << 8,
        "INPUT_VOLTAGE": 1 << 9,
        "MOTOR_ERROR_FLAG": 1 << 16,
        "MOTOR_POSITION": 1 << 17,
        "MOTOR_ID": 1 << 18,
    },
    "COMM_PACKET_ID": {
        "COMM_FW_VERSION": 0,
        "COMM_JUMP_TO_BOOTLOADER": 1,
        "COMM_ERASE_NEW_APP": 2,
        "COMM_WRITE_NEW_APP_DATA": 3,
        "COMM_GET_VALUES": 4,
        "COMM_SET_DUTY": 5,
        "COMM_SET_CURRENT": 6,
        "COMM_SET_CURRENT_BRAKE": 7,
        "COMM_SET_RPM": 8,
        "COMM_SET_POS": 9,
        "COMM_SET_HANDBRAKE": 10,
        "COMM_SET_DETECT": 11,
        "COMM_ROTOR_POSITION": 22,
        "COMM_GET_VALUES_SETUP": 50,
        "COMM_SET_POS_SPD": 91,
        "COMM_SET_POS_MULTI": 92,
        "COMM_SET_POS_SINGLE": 93,
        "COMM_SET_POS_UNLIMITED": 94,
        "COMM_SET_POS_ORIGIN": 95,
    },
    "AK80-9": {
        "P_min": -32000,  # -3200 deg
        "P_max": 32000,  # 3200 deg
        "V_min": -32000,  # -320000 rpm electrical speed
        "V_max": 32000,  # 320000 rpm electrical speed
        "Curr_min": -1500,  # -60A is the acutal limit but set to -15A
        "Curr_max": 1500,  # 60A is the acutal limit but set to 15A
        "T_min": -30,  # NM
        "T_max": 30,  # NM
        "Kt_TMotor": 0.091,  # from TMotor website (actually 1/Kvll)
        "Current_Factor": 0.59,
        "Kt_actual": 0.115,
        "GEAR_RATIO": 9.0,
        "Use_derived_torque_constants": False,  # true if you have a better model
    },
}


def buffer_append_int16(buffer, number):
    """
    buffer size for int 16

    Args:
        Buffer: memory allocated to store data.
        number: value.
        index: Size of the buffer.
    """
    buffer.append((number >> 8) & (0x00FF))
    buffer.append((number) & (0x00FF))


# Buffer allocation for unsigned 16 bit
def buffer_append_uint16(buffer, number):
    """
    buffer size for Uint 16

    Args:
        Buffer: memory allocated to store data.
        number: value.
        index: Size of the buffer.
    """
    buffer.append((number >> 8) & (0x00FF))
    buffer.append((number) & (0x00FF))


# Buffer allocation for 32 bit
def buffer_append_int32(buffer, number):
    """
    buffer size for int 32

    Args:
        Buffer: memory allocated to store data.
        number: value.
        index: Size of the buffer.
    """
    buffer.append((number >> 24) & (0x000000FF))
    buffer.append((number >> 16) & (0x000000FF))
    buffer.append((number >> 8) & (0x000000FF))
    buffer.append((number) & (0x000000FF))


# Buffer allocation for 32 bit
def buffer_append_uint32(buffer, number):
    """
    buffer size for uint 32

    Args:
        Buffer: memory allocated to store data.
        number: value.
        index: Size of the buffer.
    """
    buffer.append((number >> 24) & (0x000000FF))
    buffer.append((number >> 16) & (0x000000FF))
    buffer.append((number >> 8) & (0x000000FF))
    buffer.append((number) & (0x000000FF))


# Buffer allocation for 64 bit
def buffer_append_int64(buffer, number):
    """
    buffer size for int 64

    Args:
        Buffer: memory allocated to store data.
        number: value.
        index: Size of the buffer.
    """
    buffer.append((number >> 56) & (0x00000000000000FF))
    buffer.append((number >> 48) & (0x00000000000000FF))
    buffer.append((number >> 40) & (0x00000000000000FF))
    buffer.append((number >> 31) & (0x00000000000000FF))
    buffer.append((number >> 24) & (0x00000000000000FF))
    buffer.append((number >> 16) & (0x00000000000000FF))
    buffer.append((number >> 8) & (0x00000000000000FF))
    buffer.append((number) & (0x00000000000000FF))


# Buffer allocation for Unsigned 64 bit
def buffer_append_uint64(buffer, number):
    """
    buffer size for uint 64

    Args:
        Buffer: memory allocated to store data.
        number: value.
        index: Size of the buffer.
    """
    buffer.append((number >> 56) & (0x00000000000000FF))
    buffer.append((number >> 48) & (0x00000000000000FF))
    buffer.append((number >> 40) & (0x00000000000000FF))
    buffer.append((number >> 31) & (0x00000000000000FF))
    buffer.append((number >> 24) & (0x00000000000000FF))
    buffer.append((number >> 16) & (0x00000000000000FF))
    buffer.append((number >> 8) & (0x00000000000000FF))
    buffer.append((number) & (0x00000000000000FF))


def buffer_get_int8(data, ind):
    return np.int8((data[ind] << 8))


def buffer_get_int16(data, ind):
    return np.int16((np.uint8(data[ind]) << 8) | np.uint8(data[ind + 1]))


def buffer_get_int32(data, ind):
    return np.int32(
        (np.uint8(data[ind]) << 24)
        | (np.uint8(data[ind + 1]) << 16)
        | (np.uint8(data[ind + 2]) << 8)
        | np.uint8(data[ind + 3])
    )


class servo_motor_serial_state:
    def __init__(self):
        self.initialized = False
        self.mos_temperature = 0
        self.motor_temperature = 0
        self.output_current = 0
        self.input_current = 0
        self.id_current = 0
        self.iq_current = 0
        self.duty = 0
        self.speed = 0
        self.input_voltage = 0
        self.position = 0
        self.controlID = 0
        self.Vd = 0
        self.Vq = 0
        self.error = 0

    def __str__(self):
        s = f"Mos Temp: {self.mos_temperature}"
        s += f"\nMotor Temp: {self.motor_temperature}"
        s += f"\nOutput Current: {self.output_current}"
        s += f"\nInput Current: {self.input_current}"
        s += f"\nid current: {self.id_current}"
        s += f"\niq current: {self.iq_current}"
        s += f"\nduty: {self.duty}"
        s += f"\nspeed: {self.speed}"
        s += f"\ninput voltage: {self.input_voltage}"
        s += f"\nposition: {self.position}"
        s += f"\ncontrolID: {self.controlID}"
        s += f"\nVd: {self.Vd}"
        s += f"\nVq: {self.Vq}"
        return s


def crc16(data, DL):
    cksum = np.uint16(0)
    for i in range(DL):
        # From C example code:
        # cksum = crc16_tab[(((cksum >> 8) ^ *buf++) & 0xFF)] ^ (cksum << 8)
        cksum = crc16_tab[((cksum >> 8) ^ (data[i])) & 0xFF] ^ (cksum << 8)
    return np.uint16(cksum)


def create_frame(data):
    frame = []
    DL = len(data)
    if DL > 256:
        raise RuntimeError("Tried to send packet longer than 256 bytes!")
    else:
        frame.append(0x02)
        frame.append(DL)
        frame += data
        crc = crc16(data, DL)
        frame.append(np.uint8(crc >> 8))
        frame.append(np.uint8(crc & 0xFF))
        frame.append(0x03)
    return frame


def parse_frame(frame):
    # add error checking later
    header = frame[0]
    DL = frame[1]
    data = frame[2 : 2 + DL]
    crc = frame[2 + DL : DL + 4]
    footer = frame[-1]
    return data


def set_motor_parameter_return_format_all():
    header = Servo_Params_Serial["COMM_PACKET_ID"]["COMM_GET_VALUES_SETUP"]
    data = [header, 0xFF, 0xFF, 0xFF, 0xFF]
    return create_frame(data)


def get_motor_parameters():
    header = Servo_Params_Serial["COMM_PACKET_ID"]["COMM_GET_VALUES"]
    data = [header]
    return create_frame(data)


def set_duty(duty):
    buffer = []
    buffer_append_int32(buffer, int(duty * 100000.0))
    data = [Servo_Params_Serial["COMM_PACKET_ID"]["COMM_SET_DUTY"]] + buffer
    return create_frame(data)


def set_speed(speed):
    buffer = []
    buffer_append_int32(buffer, int(speed))
    data = [Servo_Params_Serial["COMM_PACKET_ID"]["COMM_SET_RPM"]] + buffer
    return create_frame(data)


def startup_sequence():
    return [0x40, 0x80, 0x20, 0x02, 0x21, 0xC0]


def set_current(current):
    buffer = []
    buffer_append_int32(buffer, int(current * 1000.0))
    data = [Servo_Params_Serial["COMM_PACKET_ID"]["COMM_SET_CURRENT"]] + buffer
    return create_frame(data)


def set_multi_turn():
    return [0x02, 0x05, 0x5C, 0x00, 0x00, 0x00, 0x00, 0x9E, 0x19, 0x03]


def read_packet(ser):
    if ser.inWaiting() < 1:
        return []
    else:
        header = ser.read(1)
        DL = int.from_bytes(ser.read(1), byteorder="big")
        i = 0
        data = []
        # print(DL)
        while i < DL and ser.inWaiting() > 0:
            data.append(int.from_bytes(ser.read(1), byteorder="big"))
            i += 1
        crc1 = int.from_bytes(ser.read(1), byteorder="big")  # check this later
        crc2 = int.from_bytes(ser.read(1), byteorder="big")
        footer = int.from_bytes(ser.read(1), byteorder="big")
        # print(data)
        # print(crc1)
        # print(crc2)
        # print(footer)
        if not (footer == 0x03):
            return []
        else:
            return data


def parse_motor_parameters(data):
    if (data[0] != Servo_Params_Serial["COMM_PACKET_ID"]["COMM_GET_VALUES"]) or (
        len(data) < 0x49
    ):
        # print("Trying to parse wrong message type")
        # print(data)
        return servo_motor_serial_state()
    else:
        state = servo_motor_serial_state()
        state.initialized = True
        i = 1
        state.mos_temperature = float(buffer_get_int16(data, i)) / 10.0
        i += 2
        state.motor_temperature = float(buffer_get_int16(data, i)) / 10.0
        i += 2
        state.output_current = float(buffer_get_int32(data, i)) / 100.0
        i += 4
        state.input_current = float(buffer_get_int32(data, i)) / 100.0
        i += 4
        state.id_current = float(buffer_get_int32(data, i)) / 100.0
        i += 4
        state.iq_current = float(buffer_get_int32(data, i)) / 100.0
        i += 4
        state.duty = float(buffer_get_int16(data, i)) / 1000.0
        i += 2
        state.speed = float(buffer_get_int32(data, i))
        i += 4
        state.input_voltage = float(buffer_get_int16(data, i)) / 10.0
        i += 2 + 24
        state.error = float(np.uint(data[i]))
        i += 1
        state.position = float(buffer_get_int32(data, i)) / 1000000.0
        i += 4
        state.controlID = np.uint(data[i])
        i += 1 + 6
        state.Vd = float(buffer_get_int32(data, i)) / 1000.0
        i += 4
        state.Vq = float(buffer_get_int32(data, i)) / 1000.0
        i += 4
        return state


def hex_print(arr):
    print([hex(d) for d in arr])


def stream_serial_data(end_time=5):
    with serial.Serial("/dev/ttyUSB0", 961200, timeout=100) as ser:
        loop = SoftRealtimeLoop(dt=0.1, report=True, fade=0.0)
        ser.write(bytearray(startup_sequence()))

        # not sure how to get position out of -360 to 360 degrees
        ser.write(bytearray(set_motor_parameter_return_format_all()))

        print("\n\n\n\n\n\n\n\n\n")
        for t in loop:
            if t > end_time:
                # print(["\n"]*14)
                return
            else:
                data = read_packet(ser)
                if len(data):
                    # print(data)
                    # hex_print(data)
                    print()
                    print(parse_motor_parameters(data), end="")
                    print("\x1b[13A", end="")

                cmd = get_motor_parameters()
                # hex_print(cmd)
                # print(bytearray(cmd))
                ser.write(bytearray(cmd))


if __name__ == "__main__":
    # print(hex(crc16([14],1)))
    # print(hex(crc16([4],1)))

    # F = create_frame([0x05, 0x00, 0x00, 0x4E, 0x20])
    # print([hex(f) for f in F])
    # P = parse_frame(F)
    # print([hex(p) for p in P])
    # hex_print(set_motor_parameter_return_format_all())
    # hex_print(get_motor_parameters())
    # frame = [0x02 , 0x49 , 0x04 , 0x01 , 0x66 , 0xFC , 0xD0 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 ,
    #         0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 ,
    #         0x00 , 0xFF , 0xFF , 0xFF , 0xF3 , 0x00 , 0xF6 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 ,
    #         0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0xFF,
    #         0xFF , 0xFF , 0xFF , 0x00 , 0x16 , 0xD7 , 0xAD , 0x00 , 0x0A , 0x6F , 0x19 , 0x40 ,
    #         0x7E , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 ,
    #         0x00 , 0x00 , 0x04 , 0x4D , 0x53 , 0x03 , 0x02,0x05 , 0x16 , 0x00 , 0x1A , 0xB6 ,
    #         0x03 , 0xC9 , 0xB5 , 0x03] # from manual
    # print(parse_motor_parameters(parse_frame(frame)))
    # hex_print(set_duty(0.2))
    # hex_print(set_duty(-0.2))
    # hex_print(set_current(5))
    # hex_print(set_current(-5))
    # hex_print(set_speed(1000))
    # hex_print(set_speed(-1000))
    stream_serial_data(100)
