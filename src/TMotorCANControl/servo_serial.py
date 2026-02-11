import serial
from serial import threaded
import time
import numpy as np
from enum import Enum
import traceback
import csv

"""
A python module to control the CubeMars AK series actuators over the serial port.
"""

Servo_Params_Serial = {
    "AK80-9": {
        "Type": "AK80-9",  # name of motor to print out in diagnostics
        "P_min": -58.85,  # rad (-58.85 rad limit)
        "P_max": 58.85,  # rad (58.85 rad limit)
        "V_min": -20.0,  # rad/s (-58.18 rad/s limit)
        "V_max": 20.0,  # rad/s (58.18 rad/s limit)
        "Curr_min": -15.0,  # A (-60A is the acutal limit)
        "Curr_max": 15.0,  # A (60A is the acutal limit)
        "Temp_max": 40.0,  # max mosfet temp in deg C
        "Kt": 0.115,  # Nm before gearbox per A of qaxis current
        "GEAR_RATIO": 9,  # 9:1 gear ratio
        "NUM_POLE_PAIRS": 21,  # 21 pole pairs
    },
    "AK40-10": {
        "Type": "AK40-10",  # name of motor to print out in diagnostics
        "P_min": -58.85,  # rad (-58.85 rad limit)
        "P_max": 58.85,  # rad (58.85 rad limit)
        "V_min": -90.0,  # rad/s
        "V_max": 90.0,  # rad/s
        "Curr_min": -15.0,  # A
        "Curr_max": 15.0,  # A
        "Temp_max": 50.0,  # max mosfet temp in deg C
        "Kt": 0.071,  # Nm before gearbox per A of qaxis current
        "GEAR_RATIO": 10,  # 10:1 gear ratio
        "NUM_POLE_PAIRS": 7,  # 7 pole pairs
    },
}
"""
Dictionary with the default parameters for the motors, indexed by their name

Parameters:
    Type (str): the name of this type of motor (ie, AK##-##)
    P_min (float): Minimum position in radians
    P_max (float): Maximum position in radians
    V_min (float): Minimum speed command in velocity control in rad/s
    V_max (float): Maximum speed command in velocity control in rad/s
    Curr_min (float): Minimum current command during current control in A
    Curr_max (float): Maximum current command during current control in A
    Temp_max (float): Temperature above which motor should be turned off in Celsius
    Kt (float): Torque constant of motor in Nm/A, before gearbox, q-axis
    GEAR_RATIO (int): The gear ratio of the motor "n" as in n:1
    NUM_POLE_PAIRS (int): Number of pole pairs in the motor
"""

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
"""
CRC 16 table as given in the manual, to be used to verify the packets recived
"""

PARAMETER_FLAGS = {
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
}
"""
A dictionary of flags, where 1 means enable the desired parameter during feedback, and 0 means disable.
"""

ERROR_CODES = {
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
}
"""
A dictionary mapping error code numbers to the name of the error code
"""


class COMM_PACKET_ID:
    """
    Basically used as ENUM to identify what each packet ID does
    """

    COMM_FW_VERSION = 0
    COMM_JUMP_TO_BOOTLOADER = 1
    COMM_ERASE_NEW_APP = 2
    COMM_WRITE_NEW_APP_DATA = 3
    COMM_GET_VALUES = 4
    COMM_SET_DUTY = 5
    COMM_SET_CURRENT = 6
    COMM_SET_CURRENT_BRAKE = 7
    COMM_SET_RPM = 8
    COMM_SET_POS = 9
    COMM_SET_HANDBRAKE = 10
    COMM_SET_DETECT = 11
    # COMM_GET_POS = 16
    COMM_ROTOR_POSITION = 22
    COMM_GET_VALUES_SETUP = 50
    COMM_SET_POS_SPD = 91
    COMM_SET_POS_MULTI = 92
    COMM_SET_POS_SINGLE = 93
    COMM_SET_POS_UNLIMITED = 94
    COMM_SET_POS_ORIGIN = 95


def buffer_append_int16(buffer, number):
    """
    split a 16 bit signed integer into 2 bytes and append to buffer.

    Args:
        Buffer: memory allocated to store data.
        number: value.
    """
    buffer.append((number >> 8) & (0x00FF))
    buffer.append((number) & (0x00FF))


# Buffer allocation for unsigned 16 bit
def buffer_append_uint16(buffer, number):
    """
    split a 16 bit unsigned integer into 2 bytes and append to buffer.

    Args:
        Buffer: memory allocated to store data.
        number: value.
    """
    buffer.append((number >> 8) & (0x00FF))
    buffer.append((number) & (0x00FF))


# Buffer allocation for 32 bit
def buffer_append_int32(buffer, number):
    """
    split a 32 bit signed integer into 4 bytes and append to buffer.

    Args:
        Buffer: memory allocated to store data.
        number: value.
    """
    buffer.append((number >> 24) & (0x000000FF))
    buffer.append((number >> 16) & (0x000000FF))
    buffer.append((number >> 8) & (0x000000FF))
    buffer.append((number) & (0x000000FF))


# Buffer allocation for 32 bit
def buffer_append_uint32(buffer, number):
    """
    split a 32 bit unsigned integer into 4 bytes and append to buffer.

    Args:
        Buffer: memory allocated to store data.
        number: value.
    """
    buffer.append((number >> 24) & (0x000000FF))
    buffer.append((number >> 16) & (0x000000FF))
    buffer.append((number >> 8) & (0x000000FF))
    buffer.append((number) & (0x000000FF))


# Buffer allocation for 64 bit
def buffer_append_int64(buffer, number):
    """
    split a 64 bit signed integer into 8 bytes and append to buffer.

    Args:
        Buffer: memory allocated to store data.
        number: value.
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
    split a 64 bit unsigned integer into 8 bytes and append to buffer.

    Args:
        Buffer: memory allocated to store data.
        number: value.
    """
    buffer.append((number >> 56) & (0x00000000000000FF))
    buffer.append((number >> 48) & (0x00000000000000FF))
    buffer.append((number >> 40) & (0x00000000000000FF))
    buffer.append((number >> 31) & (0x00000000000000FF))
    buffer.append((number >> 24) & (0x00000000000000FF))
    buffer.append((number >> 16) & (0x00000000000000FF))
    buffer.append((number >> 8) & (0x00000000000000FF))
    buffer.append((number) & (0x00000000000000FF))


def buffer_get_int8(buffer, ind):
    """
    Grab the 8 bit integer at data[ind]

    Args:
        buffer: array with bytes of data
        ind: location to index

    Returns:
        8 bit integer at data[ind]
    """
    return np.int8((buffer[ind]))


def buffer_get_int16(buffer, ind):
    """
    Grab the 16 bit integer at data[ind:ind+1]

    Args:
        buffer: array with bytes of data
        ind: location to index

    Returns:
        16 bit integer at data[ind:ind+1]
    """
    return np.int16((np.uint8(buffer[ind]) << 8) | np.uint8(buffer[ind + 1]))


def buffer_get_int32(buffer, ind):
    """
    Grab the 32 bit integer at data[ind:ind+3]

    Args:
        buffer: array with bytes of data
        ind: location to index

    Returns:
        32-bit signed integer at data[ind:ind+3]
    """
    return np.int32(
        (np.uint8(buffer[ind]) << 24)
        | (np.uint8(buffer[ind + 1]) << 16)
        | (np.uint8(buffer[ind + 2]) << 8)
        | np.uint8(buffer[ind + 3])
    )


def crc16(data, DL):
    """
    Calculate the crc16 value for the given data array and data length.
    This is just translated to python from the C code in the manual on the cubmars website.

    Args:
        data: array of data to creat checksum value for
        DL: data length

    Returns:
        16-bit integer checksum.
    """
    cksum = np.uint16(0)
    for i in range(DL):
        cksum = crc16_tab[((cksum >> 8) ^ (data[i])) & 0xFF] ^ (cksum << 8)
    return np.uint16(cksum)


def create_packet(data):
    """
    Packages the data into a packet to be sent.

    Args:
        data: array of N+1 bytes of data, [packet id, data_1, ..., data_N]

    Returns:
        packet: array in the format, [0x02, data length, packet id, data_1, ..., data_N, crc_1, crc_2, 0x03]
    """
    packet = []
    DL = len(data)
    if DL > 256:
        raise RuntimeError("Tried to send packet longer than 256 bytes!")
    else:
        packet.append(0x02)
        packet.append(DL)
        packet += data
        crc = crc16(data, DL)
        packet.append(np.uint8(crc >> 8))
        packet.append(np.uint8(crc & 0xFF))
        packet.append(0x03)
    return packet


def parse_packet(packet):
    """
    Check that the packet makes sense, and get the data out of it.

    Args:
        packet: array in the format [0x02, data length, packet id, data_1, ..., data_N, crc_1, crc_2, 0x03]

    Returns:
        data: Just the packet id and actual data from the recieved packet, [packet id, data_1, ..., data_N]
        None: returns None if the packet failed to successfully parse
    """
    if len(packet) > 4:
        header = packet[0]
        DL = packet[1]
        data = packet[2 : 2 + DL]
        crc = buffer_get_int16(packet[2 + DL : DL + 4], 0)
        if crc == crc16(data, DL):
            return data
        else:
            return None
    else:
        return None


class servo_serial_motor_state:
    """
    An object representing the state of the motor
    """

    def __init__(self):
        """
        Initialize the motor state to zero.
        """
        self.mos_temperature = 0
        self.motor_temperature = 0
        self.output_current = 0
        self.input_current = 0
        self.id_current = 0
        self.iq_current = 0
        self.duty = 0
        self.speed = 0
        self.input_voltage = 0
        self.position_set = 0
        self.controlID = 0
        self.Vd = 0
        self.Vq = 0
        self.error = 0
        self.acceleration = 0
        self.position = 0

    def set_state(
        self,
        mos_temperature=None,
        motor_temperature=None,
        output_current=None,
        input_current=None,
        id_current=None,
        iq_current=None,
        duty=None,
        speed=None,
        input_voltage=None,
        position_set=None,
        controlID=None,
        Vd=None,
        Vq=None,
        error=None,
        acceleration=None,
        position=None,
    ):
        """
        Set the motor state based on input to the function. If any field is not specified,
        then that field will not be altered.
        """
        self.mos_temperature = (
            mos_temperature if not (mos_temperature is None) else self.mos_temperature
        )
        self.motor_temperature = (
            motor_temperature
            if not (motor_temperature is None)
            else self.motor_temperature
        )
        self.output_current = (
            output_current if not (output_current is None) else self.output_current
        )
        self.input_current = (
            input_current if not (input_current is None) else self.input_current
        )
        self.id_current = id_current if not (id_current is None) else self.id_current
        self.iq_current = iq_current if not (iq_current is None) else self.iq_current
        self.duty = duty if not (duty is None) else self.duty
        self.speed = speed if not (speed is None) else self.speed
        self.input_voltage = (
            input_voltage if not (input_voltage is None) else self.input_voltage
        )
        self.position_set = (
            position_set if not (position_set is None) else self.position_set
        )
        self.controlID = controlID if not (controlID is None) else self.controlID
        self.Vd = Vd if not (Vd is None) else self.Vd
        self.Vq = Vq if not (Vq is None) else self.Vq
        self.error = error if not (error is None) else self.error
        self.acceleration = (
            acceleration if not (acceleration is None) else self.acceleration
        )
        self.position = position if not (position is None) else self.position

    def __str__(self):
        """
        String showing each of the fields in the motor state.
        """
        s = f"Mos Temp: {self.mos_temperature}"
        s += f"\nMotor Temp: {self.motor_temperature}"
        s += f"\nOutput Current: {self.output_current}"
        s += f"\nInput Current: {self.input_current}"
        s += f"\nid current: {self.id_current}"
        s += f"\niq current: {self.iq_current}"
        s += f"\nduty: {self.duty}"
        s += f"\nspeed: {self.speed}"
        s += f"\ninput voltage: {self.input_voltage}"
        s += f"\nposition: {self.position_set}"
        s += f"\ncontrolID: {self.controlID}"
        s += f"\nVd: {self.Vd}"
        s += f"\nVq: {self.Vq}"
        s += f"\nAccel: {self.acceleration}"
        return s


# possible states for the controller
class SERVO_SERIAL_CONTROL_STATE(Enum):
    """
    An Enum to keep track of different control states
    """

    DUTY_CYCLE = 0
    CURRENT_LOOP = 1
    CURRENT_BRAKE = 2
    VELOCITY = 3
    POSITION = 4
    HANDBRAKE = 5
    POSITION_VELOCITY = 6
    IDLE = 7


class motor_listener(serial.threaded.Protocol):
    """
    Implements the pyserial "Protocol" class to handle messages asynchronously
    TODO when pyserial implmements asyncio support, switch to that
    """

    def connection_made(self, transport):
        """
        Could add other things to happen here on connection initialization.

        Args:
            transport: the connection
        """
        super().connection_made(transport)

    def connection_lost(self, transport):
        """
        Could add other things to happen here on connection termination.

        Args:
            transport: the connection
        """
        super().connection_made(transport)

    def __init__(self):
        """
        Initializes the class, including the state machine variables and a reference
        to the motor manager in the main thread that this class will update.
        """
        super().__init__()
        self.buffer = []
        self.state = 0  # 0 : ready, 1 : message started (0x02), 2 : known DL, 3 : ready data, 5 : expect 0x03
        self.DL = 0
        self.i = 0
        self.motor = None

    def data_received(self, data):
        """
        Handle data that's been recieved, by stepping through a state machine one byte at a time.
        States:
            0: expecting 0x02 for beginning of next message
            1: expecting data length field
            2: will keep reading data until DL + 2
            3: expecting 0x03 for ending of this message

        Args:
            data: array of received data to parse
        """

        for d in data:
            if self.state == 0:
                if d == 0x02:
                    self.buffer.append(d)
                    self.state = 1
            elif self.state == 1:
                self.buffer.append(d)
                self.DL = d
                self.state = 2
            elif self.state == 2:
                self.buffer.append(d)
                self.i += 1
                if self.i == self.DL + 2:
                    self.state = 3
                    self.i = 0
                    self.DL = 0
            elif self.state == 3:
                if d == 0x03:
                    self.buffer.append(d)
                    self.handle_packet(self.buffer)
                    self.state = 0
                    self.buffer = []

    def handle_packet(self, packet):
        """
        Called whenever the state machine finishes reading a packet,
        to update motor manager object.

        Args:
            packet: array of data to parse.
        """
        if self.motor is not None:
            DL = packet[1]
            data = packet[2 : 2 + DL]
            crc = buffer_get_int16(packet[2 + DL : DL + 4], 0)
            if crc != crc16(data, DL):
                data = None
            if not (data is None) and len(data) > 0:
                self.motor.update_async(data)


# the user-facing class that manages the motor.
class TMotorManager_servo_serial:
    """
    The user-facing class that manages the motor. This class should be
    used in the "context" of a with as block, in order to safely enter/exit
    control of the motor.
    """

    def __init__(
        self,
        port="/dev/ttyUSB0",
        baud=921600,
        motor_params=Servo_Params_Serial["AK80-9"],
        max_mosfett_temp=50,
    ):
        """
        Initialize the motor manager. Note that this will not turn on the motor,
        until __enter__ is called (automatically called in a with block)

        Args:
            port: the name of the serial port to connect to (ie, /dev/ttyUSB0, COM3, etc)
            baud: the baud rate to use for connection. Should always be 921600 as far as I can tell.
            motor_params: A parameter dictionary defining the motor parameters, as defined above.
            max_mosfett_temp: Temperature of the mosfett above which to throw an error, in Celsius
        """
        self.motor_params = motor_params
        self.port = port
        self.baud = baud
        print("Initializing device: " + self.device_info_string())

        self._motor_state = servo_serial_motor_state()
        self._motor_state_async = servo_serial_motor_state()
        self._command = None  # overwrite with byte array of command to send on update()
        self._control_state = None
        self.max_temp = max_mosfett_temp  # max temp in deg C, can update later

        # TODO verify these work for other motor types!
        self.radps_per_ERPM = 2 * np.pi / 180 / 60  # 5.82E-04
        self.rad_per_Eang = (
            2 * (np.pi / 180) / (self.motor_params["NUM_POLE_PAIRS"])
        )  # 1.85e-4

        self._entered = False
        self._start_time = time.time()
        self._last_update_time = self._start_time
        self._updated_async = False
        self._updated = False
        self._ser = None
        self._reader_thread = None

    def __enter__(self):
        """
        Used to safely power the motor on.
        """
        if not self._entered:
            # begin serial connection
            self._ser = serial.Serial(
                self.port,
                self.baud,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=0.0001,
                inter_byte_timeout=0.001,
            )

            # start another thread to handle recieved data
            self._reader_thread = serial.threaded.ReaderThread(
                self._ser, motor_listener
            )
            self._listener = self._reader_thread.__enter__()
            self._listener.motor = self

            # send startup sequence
            self.power_on()
            self.send_command()

            # tell the motor to send back all parameters
            # TODO expand to allow user to only request some data, could be faster
            self.comm_set_motor_parameter_return_format_all()
            self.send_command()

            # tell motor to send current position (for some reason current position is not in the other parameters)
            self.comm_begin_position_feedback()
            self.send_command()

            # don't do anything else
            self.enter_idle_mode()
            self.send_command()

            if not self.check_connection():
                print(f"device: {self.device_info_string()} not connected!")
            else:
                print(f"device: {self.device_info_string()} successfully connected.")

            # return the safely entered motor manager object
            self._entered = True
            return self
        else:
            # it's already been entered!
            print(f"Already entered device: {self.device_info_string()}")

    def __exit__(self, etype, value, tb):
        """
        Used to safely power the motor off and close the reader thread and serial port.
        """
        print("\nTurning off control for device: " + self.device_info_string())
        # end reading thread
        self._reader_thread.stop()

        # power down motor
        self._ser.write(self.comm_set_duty_cycle(0.0))

        # end serial connection
        self._ser.close()

        # if this was ended due to an error, print the stack trace
        if not (etype is None):
            traceback.print_exception(etype, value, tb)

    def check_connection(self):
        """
        For now, just sends some parameter read commands and waits 0.2 seconds to
        see if we got a response.
        """
        self._send_specific_command(self.comm_get_motor_parameters())
        self._send_specific_command(self.comm_get_motor_parameters())
        self._send_specific_command(self.comm_get_motor_parameters())
        # slight delay to ensure connection!
        time.sleep(0.2)
        return self._updated_async

    def update_async(self, data):
        """
        Update the asynchronous motor state. Called by a reader thread.

        Args:
            data: An array of N bytes of data to parse, [packet id, data_1, ..., data_N]

        Raises:
            RuntimeError: If the packet recieved contains an error code.
        """
        self._updated_async = True
        packet_ID = data[0]
        if packet_ID in [
            COMM_PACKET_ID.COMM_GET_VALUES,
            COMM_PACKET_ID.COMM_GET_VALUES_SETUP,
        ]:
            self.parse_motor_parameters_async(data)
            # calculate acceleration
            now = time.time()
            dt = now - self._last_update_time
            self._motor_state_async.acceleration = self._motor_state_async.speed / dt
            self._last_update_time = now
        elif packet_ID == COMM_PACKET_ID.COMM_ROTOR_POSITION:
            self.parse_position_feedback_async(data)
        else:
            print("weird packet")
        # elif (packet_ID == COMM_PACKET_ID.COMM_SET_POS):
        #     self.parse_set_position_feedback_async(data)

        if self._motor_state.error != 0:
            raise RuntimeError(ERROR_CODES[self._motor_state.error])

    # comm data parsing
    def parse_position_feedback_async(self, data):
        """
        Update this motor's asynch position based on recieved data

        Args:
            data: The data array to parse the position from.
        """
        self._motor_state_async.position = -float(buffer_get_int32(data, 1)) / 1000.0

    def parse_set_position_feedback_async(self, data):
        """
        Update this motor's asynch position based on recieved data.

        Args:
            data: The data array to parse the position from.
        """
        # not sure why the constant varies here, maybe the streaming data is more accurate?
        self._motor_state_async.position = -float(buffer_get_int32(data, 1)) / 1000000.0

    def parse_motor_parameters_async(self, data):
        """
        Update this motor's asynch state (except position) based on received data

        Args:
            data: The data array to parse the parameters from
        """
        i = 1
        self._motor_state_async.mos_temperature = (
            float(buffer_get_int16(data, i)) / 10.0
        )
        i += 2
        self._motor_state_async.motor_temperature = (
            float(buffer_get_int16(data, i)) / 10.0
        )
        i += 2
        self._motor_state_async.output_current = (
            float(buffer_get_int32(data, i)) / 100.0
        )
        i += 4
        self._motor_state_async.input_current = float(buffer_get_int32(data, i)) / 100.0
        i += 4
        self._motor_state_async.id_current = float(buffer_get_int32(data, i)) / 100.0
        i += 4
        self._motor_state_async.iq_current = float(buffer_get_int32(data, i)) / 100.0
        i += 4
        self._motor_state_async.duty = float(buffer_get_int16(data, i)) / 1000.0
        i += 2
        self._motor_state_async.speed = float(buffer_get_int32(data, i))
        i += 4
        self._motor_state_async.input_voltage = float(buffer_get_int16(data, i)) / 10.0
        i += 2 + 24
        # TODO investigate what's in the 24 reserved bytes? Maybe it's interesting to record?
        self._motor_state_async.error = np.uint(data[i])
        i += 1
        self._motor_state_async.position_set = (
            float(buffer_get_int32(data, i)) / 1000000.0
        )
        i += 4
        self._motor_state_async.controlID = np.uint(data[i])
        i += 1 + 6
        self._motor_state_async.Vd = float(buffer_get_int32(data, i)) / 1000.0
        i += 4
        self._motor_state_async.Vq = float(buffer_get_int32(data, i)) / 1000.0
        i += 4

    def send_command(self):
        """
        Sends the current command that the user has specified.
        """
        if not (self._command is None):
            # use the thread-safe method to write the command, so that we don't access the serial
            # object while the reader thread is using it!!
            # TODO when pyserial adds full asyncio support, consider switching to that
            self._reader_thread.write(self._command)

    def _send_specific_command(self, command):
        """
        Sends the specified command rather than the current value of <this object>._command

        Args:
            command: bytearray with the command to send.
        """
        if not (self._command is None):
            # use the thread-safe method to write the command, so that we don't access the serial
            # object while the reader thread is using it!!
            # TODO when pyserial adds full asyncio support, consider switching to that
            self._reader_thread.write(command)

    def update(self):
        """
        Synchronizes the current motor state with the asynchronously updated state.
        Sends the current motor command
        Sends the command to get parameter feedback

        Raises:
            RuntimeError: if this method is called before the motor is entered.
        """
        if not self._entered:
            raise RuntimeError(
                "Tried to update motor state before safely powering on for device: "
                + self.device_info_string()
            )

        if self.get_temperature_celsius() > self.max_temp:
            raise RuntimeError(
                "Temperature greater than {}C for device: {}".format(
                    self.max_temp, self.device_info_string()
                )
            )

        # send the user specified command
        self.send_command()

        # send the command to get parameters (message will be read in other thread)
        self._send_specific_command(self.comm_get_motor_parameters())

        # synchronize user-facing state with most recent async state
        # TODO implement some filtering on the async state, if it gets multiple updates
        # between user requested updates
        self._motor_state = self._motor_state_async

    # comm protocol commands
    def power_on(self):
        """
        Send the startup sequence command. Not sure why it's like this, but the
        command is [0x40, 0x80, 0x20, 0x02, 0x21, 0xc0]
        """
        self._command = bytearray([0x40, 0x80, 0x20, 0x02, 0x21, 0xC0])
        self.send_command()

    def power_off(self):
        """
        There is no official power off command that I can see, so this will
        set the duty cycle to 0.0
        """
        self.set_duty_cycle(0.0)
        self.send_command()

    def enter_idle_mode(self):
        """
        Set the control state to IDLE and current command to none.
        In this mode, no command will be sent.
        """
        self._command = None
        self._control_state = SERVO_SERIAL_CONTROL_STATE.IDLE

    def enter_velocity_control(self):
        """
        Set the control state to VELOCITY
        In this mode, you can send a certain motion speed to motor
        """
        self._control_state = SERVO_SERIAL_CONTROL_STATE.VELOCITY

    def enter_position_control(self):
        """
        Set the control state to POSITION
        In this mode, you can send a certain position to motor, the motor will run to the specified
        position, (default speed 12000erpm acceleration 40000erpm)
        """
        self._control_state = SERVO_SERIAL_CONTROL_STATE.POSITION

    def enter_position_velocity_control(self):
        """
        Set the control state to POSITION_VELOCITY
        In this mode, you can send a certain position, speed and acceleration to motor.
        The motor will run at a given acceleration and maximum speed to a specified
        position.
        """
        self._control_state = SERVO_SERIAL_CONTROL_STATE.POSITION_VELOCITY

    def enter_current_control(self):
        """
        Set the control state to CURENT_LOOP
        In this mode, you can send an Iq current to motor, the motor output torque = Iq *KT, so it can
        be used as a torque loop
        """
        self._control_state = SERVO_SERIAL_CONTROL_STATE.CURRENT_LOOP

    def enter_duty_cycle_control(self):
        """
        Set the control state to DUTY_CYCLE
        In this mode, you can send a certain duty cycle voltage to motor
        """
        self._control_state = SERVO_SERIAL_CONTROL_STATE.DUTY_CYCLE

    def comm_set_duty_cycle(self, duty, set_command=True):
        """
        send a certain duty cycle voltage to motor

        Args:
            duty: -1.0 to 1.0 duty cycle to use
            set_command: set the TMotorManager's current command to this if True

        Returns:
            The command as a bytearray
        """
        buffer = []
        buffer_append_int32(buffer, int(duty * 100000.0))
        data = [COMM_PACKET_ID.COMM_SET_DUTY] + buffer
        if set_command:
            self._command = bytearray(create_packet(data))
        return self._command

    def comm_set_speed_ERPM(self, speed, set_command=True):
        """
        send a certain motion speed to motor

        Args:
            speed: speed to use in ERPM, sign denotes direction
            set_command: set the TMotorManager's current command to this if True

        Returns:
            The command as a bytearray
        """
        buffer = []
        buffer_append_int32(buffer, int(speed))
        data = [COMM_PACKET_ID.COMM_SET_RPM] + buffer
        if set_command:
            self._command = bytearray(create_packet(data))
        return self._command

    def comm_set_current_loop(self, current, set_command=True):
        """
        send am Iq current to motor, the motor output torque = Iq *KT, so it can
        be used as a torque loop

        Args:
            current: q axis current to use in A, sign denotes direction
            set_command: set the TMotorManager's current command to this if True

        Returns:
            The command as a bytearray
        """
        buffer = []
        buffer_append_int32(buffer, int(current * 1000.0))
        data = [COMM_PACKET_ID.COMM_SET_CURRENT] + buffer
        if set_command:
            self._command = bytearray(create_packet(data))
        return self._command

    def comm_set_position(self, pos, set_command=True):
        """
        send a certain position to motor, the motor will run to the specified
        position, (default speed 12000erpm acceleration 40000erpm)

        Args:
            pos: Desired position in degrees
            set_command: set the TMotorManager's current command to this if True

        Returns:
            The command as a bytearray
        """
        buffer = []
        buffer_append_int32(buffer, int(pos * 1000000))
        data = [COMM_PACKET_ID.COMM_SET_POS] + buffer
        if set_command:
            self._command = bytearray(create_packet(data))
        return self._command

    def comm_set_position_velocity(self, pos, vel, acc, set_command=True):
        """
        send a certain position, speed and acceleration to motor.
        The motor will run at a given acceleration and maximum speed to a specified
        position.

        Args:
            pos: Desired position in degrees
            vel: Desired speed in ERPM
            acc: Desired acceleration in ERPM/minute? TODO verify this
            set_command: set the TMotorManager's current command to this if True

        Returns:
            The command as a bytearray
        """
        # 4 byte pos * 1000, 4 byte vel * 1, 4 byte a * 1
        buffer = []
        buffer_append_int32(buffer, int(pos * 1000000))
        buffer_append_int32(buffer, int(vel))
        buffer_append_int32(buffer, int(acc))
        data = [COMM_PACKET_ID.COMM_SET_POS_SPD] + buffer
        if set_command:
            self._command = bytearray(create_packet(data))
        return self._command

    def comm_set_multi_turn(self, set_command=True):
        """
        Tell the motor to operate in multi-turn mode, rather than being limited to
        Just 360 degrees of position feedback.

        Args:
            set_command: set the TMotorManager's current command to this if True

        Returns:
            The command as a bytearray
        """
        cmd = bytearray([0x02, 0x05, 0x5C, 0x00, 0x00, 0x00, 0x00, 0x9E, 0x19, 0x03])
        if set_command:
            self._command = cmd
        return cmd

    def set_zero_position(self, set_command=True):
        """
        Set the current position of the motor to be the new zero position.

        Args:
            set_command: set the TMotorManager's current command to this if True

        Returns:
            The command as a bytearray
        """
        cmd = bytearray([0x02, 0x02, 0x5F, 0x01, 0x0E, 0xA0, 0x03])
        if set_command:
            self._command = bytearray([0x02, 0x02, 0x5F, 0x01, 0x0E, 0xA0, 0x03])
        return cmd

    def comm_set_motor_parameter_return_format_all(self, set_command=True):
        """
        Tell the motor to send back all possible fields when an update is requested.

        Args:
            set_command: set the TMotorManager's current command to this if True

        Returns:
            The command as a bytearray
        """
        header = COMM_PACKET_ID.COMM_GET_VALUES_SETUP
        data = [header, 0xFF, 0xFF, 0xFF, 0xFF]
        cmd = bytearray(create_packet(data))
        if set_command:
            self._command = cmd
        return cmd

    def comm_begin_position_feedback(self, set_command=True):
        """
        Tell the motor to send back its current position every 10ms

        Args:
            set_command: set the TMotorManager's current command to this if True

        Returns:
            The command as a bytearray
        """
        if set_command:
            self._command = bytearray([0x02, 0x02, 0x0B, 0x04, 0x9C, 0x7E, 0x03])

    def comm_get_motor_parameters(self, set_command=True):
        """
        Request the current motor parameters

        Args:
            set_command: set the TMotorManager's current command to this if True

        Returns:
            The command as a bytearray
        """
        header = COMM_PACKET_ID.COMM_GET_VALUES
        data = [header]
        packet = create_packet(data)
        cmd = bytearray(packet)

        if set_command:
            self._command = cmd
        return cmd

    # getters for motor state
    def get_temperature_celsius(self):
        """
        Returns:
        The most recently updated motor temperature in degrees C.
        """
        return self._motor_state.mos_temperature

    def get_motor_error_code(self):
        """
        Returns:
        The most recently updated motor error code.
        Note the program should throw a runtime error before you get a chance to read
        this value if it is ever anything besides 0.

        Codes:
            0  : 'FAULT_CODE_NONE'
            1  : 'FAULT_CODE_OVER_VOLTAGE'
            2  : 'FAULT_CODE_UNDER_VOLTAGE'
            3  : 'FAULT_CODE_DRIVE'
            4  : 'FAULT_CODE_ABS_OVER_CURRENT'
            5  : 'FAULT_CODE_OVER_TEMP_FET'
            6  : 'FAULT_CODE_OVER_TEMP_MOTOR'
            7  : 'FAULT_CODE_GATE_DRIVER_OVER_VOLTAGE'
            8  : 'FAULT_CODE_GATE_DRIVER_UNDER_VOLTAGE'
            9  : 'FAULT_CODE_MCU_UNDER_VOLTAGE'
            10 : 'FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET'
            11 : 'FAULT_CODE_ENCODER_SPI'
            12 : 'FAULT_CODE_ENCODER_SINCOS_BELOW_MIN_AMPLITUDE'
            13 : 'FAULT_CODE_ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE'
            14 : 'FAULT_CODE_FLASH_CORRUPTION'
            15 : 'FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1'
            16 : 'FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_2'
            17 : 'FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_3'
            18 : 'FAULT_CODE_UNBALANCED_CURRENTS'
        """
        return self._motor_state.error

    def get_motor_error_string(self):
        return ERROR_CODES[self._motor_state.error]

    def get_current_qaxis_amps(self):
        """
        Returns:
        The most recently updated qaxis current in amps
        """
        return self._motor_state.iq_current

    def get_current_daxis_amps(self):
        """
        Returns:
        The most recently updated qaxis current in amps
        """
        return self._motor_state.id_current

    def get_current_bus_amps(self):
        """
        Returns:
        The most recently updated qaxis current in amps
        """
        return self._motor_state.input_current

    def get_voltage_qaxis_volts(self):
        """
        Returns:
        The most recently updated qaxis voltage in volts
        """
        return self._motor_state.Vq

    def get_voltage_daxis_volts(self):
        """
        Returns:
        The most recently updated daxis voltage in volts
        """
        return self._motor_state.Vd

    def get_voltage_bus_volts(self):
        """
        Returns:
        The most recently updated input voltage in volts
        """
        return self._motor_state.input_current

    def get_output_angle_radians(self):
        """
        Returns:
        The most recently updated output angle in radians
        """
        return self._motor_state.position * self.rad_per_Eang

    def get_output_velocity_radians_per_second(self):
        """
        Returns:
            The most recently updated output velocity in radians per second
        """
        return self._motor_state.speed * self.radps_per_ERPM

    def get_output_acceleration_radians_per_second_squared(self):
        """
        Returns:
            The most recently updated output acceleration in radians per second per second
        """
        return self._motor_state.acceleration * self.radps_per_ERPM

    def get_output_torque_newton_meters(self):
        """
        Returns:
            the most recently updated output torque in Nm
        """
        return (
            self.get_current_qaxis_amps()
            * self.motor_params["Kt"]
            * self.motor_params["GEAR_RATIO"]
        )

    # user facing setters
    def set_output_velocity_radians_per_second(self, vel):
        """
        Update the current command to the desired velocity.
        Note, this does not send a command, it updates the TMotorManager's saved command,
        which will be sent when update() is called.

        Args:
            vel: The desired output speed in rad/s
        """
        if np.abs(vel) >= self.motor_params["V_max"]:
            raise RuntimeError(
                "Cannot control using speed mode for angles with magnitude greater than "
                + str(self.motor_params["V_max"])
                + "rad/s!"
            )

        if self._control_state not in [SERVO_SERIAL_CONTROL_STATE.VELOCITY]:
            raise RuntimeError(
                "Attempted to send speed command without entering speed control "
                + self.device_info_string()
            )

        self.comm_set_speed_ERPM(vel / self.radps_per_ERPM)

    def set_duty_cycle_percent(self, duty):
        """
        Update the current command to the desired duty cycle.
        Note, this does not send a command, it updates the TMotorManager's saved command,
        which will be sent when update() is called.

        Args:
            duty: The desired duty cycle -1.0 to 1.0
        """
        if np.abs(duty) >= 1:
            raise RuntimeError(
                "Cannot control using duty cycle mode for more than 100 percent duty!"
            )

        if self._control_state not in [SERVO_SERIAL_CONTROL_STATE.DUTY_CYCLE]:
            raise RuntimeError(
                "Attempted to duty cycle command without entering duty cycle control "
                + self.device_info_string()
            )

        self.comm_set_duty_cycle(duty)

    def set_motor_current_qaxis_amps(self, curr):
        """
        Update the current command to the desired current.
        Note, this does not send a command, it updates the TMotorManager's saved command,
        which will be sent when update() is called.

        Args:
            curr: The desired q-axis current in A
        """
        if np.abs(curr) >= self.motor_params["Curr_max"]:
            raise RuntimeError(
                "Cannot control using current mode with magnitude greater than "
                + str(self.motor_params["I_max"])
                + "rad/s!"
            )

        if self._control_state not in [SERVO_SERIAL_CONTROL_STATE.CURRENT_LOOP]:
            raise RuntimeError(
                "Attempted to send current command without entering current control "
                + self.device_info_string()
            )

        self.comm_set_current_loop(curr)

    def set_output_angle_radians(self, pos, vel=0.75, acc=0.5):
        """
        Update the current command to the desired position, when in position or position-velocity mode.
        Note, this does not send a command, it updates the TMotorManager's saved command,
        which will be sent when update() is called.

        Args:
            pos: The desired output angle in rad
            vel: The desired speed to get there in rad/s (when in POSITION_VELOCITY mode)
            acc: The desired acceleration to get there in rad/s/s, ish (when in POSITION_VELOCITY mode)
        """

        if np.abs(pos) >= self.motor_params["P_max"]:
            raise RuntimeError(
                "Cannot control using position mode for angles with magnitude greater than "
                + str(self.motor_params["P_max"])
                + "rad!"
            )
        if np.abs(vel) >= self.motor_params["V_max"]:
            raise RuntimeError(
                "Cannot control velocities with magnitude greater than "
                + str(self.motor_params["V_max"])
                + "rad/s!"
            )

        pos = pos / self.rad_per_Eang
        vel = vel / self.radps_per_ERPM
        acc = acc / self.radps_per_ERPM
        if self._control_state == SERVO_SERIAL_CONTROL_STATE.POSITION_VELOCITY:
            self.comm_set_position_velocity(pos, vel, acc)
        elif self._control_state == SERVO_SERIAL_CONTROL_STATE.POSITION:
            self.comm_set_position(pos)
        else:
            raise RuntimeError(
                "Attempted to send position command without entering position control "
                + self.device_info_string()
            )

    def set_output_torque_newton_meters(self, torque):
        """
        Update the current command to the desired current, based on the requested torque.
        Note, this does not send a command, it updates the TMotorManager's saved command,
        which will be sent when update() is called.

        Args:
            torque: The desired output torque in Nm.
        """
        self.set_motor_current_qaxis_amps(
            (torque / self.motor_params["Kt"] / self.motor_params["GEAR_RATIO"])
        )

    # motor-side functions to account for the gear ratio
    def set_motor_torque_newton_meters(self, torque):
        """
        Version of set_output_torque that accounts for gear ratio to control motor-side torque

        Args:
            torque: The desired motor-side torque in Nm.
        """
        self.set_output_torque_newton_meters(torque * self.motor_params["Kt"])

    def set_motor_angle_radians(self, pos):
        """
        Wrapper for set_output_angle that accounts for gear ratio to control motor-side angle

        Args:
            pos: The desired motor-side position in rad.
        """
        self.set_output_angle_radians(pos / (self.motor_params["GEAR_RATIO"]))

    def set_motor_velocity_radians_per_second(self, vel):
        """
        Wrapper for set_output_velocity that accounts for gear ratio to control motor-side velocity

        Args:
            vel: The desired motor-side velocity in rad/s.
        """
        self.set_output_velocity_radians_per_second(
            vel / (self.motor_params["GEAR_RATIO"])
        )

    def get_motor_angle_radians(self):
        """
        Wrapper for get_output_angle that accounts for gear ratio to get motor-side angle

        Returns:
            The most recently updated motor-side angle in rad.
        """
        return self._motor_state.position * self.motor_params["GEAR_RATIO"]

    def get_motor_velocity_radians_per_second(self):
        """
        Wrapper for get_output_velocity that accounts for gear ratio to get motor-side velocity

        Returns:
            The most recently updated motor-side velocity in rad/s.
        """
        return self._motor_state.velocity * self.motor_params["GEAR_RATIO"]

    def get_motor_acceleration_radians_per_second_squared(self):
        """
        Wrapper for get_output_acceleration that accounts for gear ratio to get motor-side acceleration

        Returns:
            The most recently updated motor-side acceleration in rad/s/s.
        """
        return self._motor_state.acceleration * self.motor_params["GEAR_RATIO"]

    def get_motor_torque_newton_meters(self):
        """
        Wrapper for get_output_torque that accounts for gear ratio to get motor-side torque

        Returns:
            The most recently updated motor-side torque in Nm.
        """
        return self.get_output_torque_newton_meters() * self.motor_params["GEAR_RATIO"]

    # Pretty stuff
    def __str__(self):
        """Prints the motor's device info and current state"""
        return (
            self.device_info_string()
            + " | Position: "
            + "{: 1f}".format(round(self.get_output_angle_radians(), 3))
            + " rad | Velocity: "
            + "{: 1f}".format(round(self.get_output_velocity_radians_per_second(), 3))
            + " rad/s | current: "
            + "{: 1f}".format(round(self._motor_state.iq_current, 3))
            + " A | temp: "
            + "{: 1f}".format(round(self._motor_state.mos_temperature, 0))
            + " C"
            + f" | Error: {self.get_motor_error_string()}"
        )

    def device_info_string(self):
        """Prints the motor's serial port and device type."""
        return f"{self.motor_params['Type']} Port: {self.port}"

    # controller variables
    temperature = property(get_temperature_celsius, doc="temperature_degrees_C")
    """Temperature in Degrees Celsius"""

    # TODO write actual codes in description here as well
    error = property(get_motor_error_code, doc="Error")
    """Error Codes:
            0  : 'FAULT_CODE_NONE'
            1  : 'FAULT_CODE_OVER_VOLTAGE'
            2  : 'FAULT_CODE_UNDER_VOLTAGE'
            3  : 'FAULT_CODE_DRIVE'
            4  : 'FAULT_CODE_ABS_OVER_CURRENT'
            5  : 'FAULT_CODE_OVER_TEMP_FET'
            6  : 'FAULT_CODE_OVER_TEMP_MOTOR'
            7  : 'FAULT_CODE_GATE_DRIVER_OVER_VOLTAGE'
            8  : 'FAULT_CODE_GATE_DRIVER_UNDER_VOLTAGE'
            9  : 'FAULT_CODE_MCU_UNDER_VOLTAGE'
            10 : 'FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET'
            11 : 'FAULT_CODE_ENCODER_SPI'
            12 : 'FAULT_CODE_ENCODER_SINCOS_BELOW_MIN_AMPLITUDE'
            13 : 'FAULT_CODE_ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE'
            14 : 'FAULT_CODE_FLASH_CORRUPTION'
            15 : 'FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1'
            16 : 'FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_2'
            17 : 'FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_3'
            18 : 'FAULT_CODE_UNBALANCED_CURRENTS'
    """

    # electrical variables
    current_qaxis = property(
        get_current_qaxis_amps, set_motor_current_qaxis_amps, doc="current_qaxis_amps"
    )
    """Q-axis current in amps"""

    current_daxis = property(get_current_daxis_amps, doc="current_daxis_amps")
    """D-axis current in amps"""

    current_bus = property(get_current_bus_amps, doc="current_bus_amps")
    """Bus input current in amps"""

    voltage_qaxis = property(get_voltage_qaxis_volts, doc="voltage_qaxis_volts")
    """Q-axis voltage in volts"""

    voltage_daxis = property(get_voltage_daxis_volts, doc="voltage_daxis_volts")
    """D-axis voltage in volts"""

    voltage_bus = property(get_voltage_bus_volts, doc="voltage_bus_volts")
    """Bus input voltage in volts"""

    # output-side variables
    position = property(
        get_output_angle_radians, set_output_angle_radians, doc="output_angle_radians"
    )
    """Output angle in rad"""

    velocity = property(
        get_output_velocity_radians_per_second,
        set_output_velocity_radians_per_second,
        doc="output_velocity_radians_per_second",
    )
    """Output velocity in rad/s"""

    acceleration = property(
        get_output_acceleration_radians_per_second_squared,
        doc="output_acceleration_radians_per_second_squared",
    )
    """Output acceleration in rad/s/s"""

    torque = property(
        get_output_torque_newton_meters,
        set_output_torque_newton_meters,
        doc="output_torque_newton_meters",
    )
    """Output torque in Nm"""

    # motor-side variables
    angle_motorside = property(
        get_motor_angle_radians, set_motor_angle_radians, doc="motor_angle_radians"
    )
    """Motor-side angle in rad"""

    velocity_motorside = property(
        get_motor_velocity_radians_per_second,
        set_motor_velocity_radians_per_second,
        doc="motor_velocity_radians_per_second",
    )
    """Motor-side velocity in rad/s"""

    acceleration_motorside = property(
        get_motor_acceleration_radians_per_second_squared,
        doc="motor_acceleration_radians_per_second_squared",
    )
    """Motor-side acceleration in rad/s/s"""

    torque_motorside = property(
        get_motor_torque_newton_meters,
        set_motor_torque_newton_meters,
        doc="motor_torque_newton_meters",
    )
    """Motor-side torque in Nm"""
