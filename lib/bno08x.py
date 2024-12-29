# BNO08X Micropython I2C Library by Dobodu
#
# Adapted from original Adafruit CircuitPyhton library
# SPDX-FileCopyrightText: Copyright (c) 2020 Bryan Siepert for Adafruit Industries
# SPDX-License-Identifier: MIT
#
# TODO : (From original Library)
#
# Calibrated Acceleration (m/s2)
# Calibration
# Raw ACCEL, MAG, GYRO

from ustruct import unpack_from, pack_into
from collections import namedtuple
from math import asin, atan2, degrees
from utime import ticks_ms, sleep_ms, ticks_diff

LIBNAME = "BNO08X"
LIBVERSION = "1.0.1"

#BNO08X SETUP
BNO08X_DEFAULT_ADDRESS = (0x4A, 0x4B)

#Buffer Size
DATA_BUFFER_SIZE = 1024

# Channel Numbers
BNO_CHANNEL_SHTP_COMMAND = 0x00
BNO_CHANNEL_EXE = 0x01
BNO_CHANNEL_CONTROL = 0x02
BNO_CHANNEL_INPUT_SENSOR_REPORTS = 0x03
BNO_CHANNEL_WAKE_INPUT_SENSOR_REPORTS = 0x04
BNO_CHANNEL_GYRO_ROTATION_VECTOR = 0x05

#Configuring Reports
COMMAND_RESPONSE = 0xF1
COMMAND_REQUEST = 0xF2
FRS_READ_RESPONSE = 0xF3
FRS_READ_REQUEST = 0xF4
FRS_WRITE_RESPONSE = 0xF5
FRS_WRITE_DATA = 0xF6
FRS_WRITE_REQUEST = 0xF7
SHTP_REPORT_ID_RESPONSE = 0xF8
SHTP_REPORT_ID_REQUEST = 0xF9
REBASE_TIMESTAMP = 0xFA
BASE_TIMESTAMP = 0xFB
GET_FEATURE_RESPONSE = 0xFC
SET_FEATURE_COMMAND = 0xFD
GET_FEATURE_REQUEST = 0xFE

# DCD/ ME Commands
ME_ERRORREPORT_CDE = 0x01
ME_COUNTER_CDE = 0x02
ME_TARE_CDE = 0x03
ME_INIT_CDE = 0x04
ME_SAVE_DCD_CDE = 0x06
ME_CALIBRATION_CDE = 0x07
ME_SAVE_DCD_PERIODIC_CDE = 0x09
ME_OSCILLATOR_TYPE_CDE = 0x0A
ME_RESET_DCD_CDE = 0x0B

#DCD/ME Sub-commands
ME_COUNTER_GETCOUNTS_CDE = 0x00
ME_COUNTER_CLEARCOUNTS_CDE = 0x01
ME_TARE_NOW_SUBCDE = 0x00
ME_TARE_PERSIST_SUBCDE = 0x01
ME_TARE_REORIENTATION_SUBCDE = 0x02
ME_CALIBRATION_CONFIG_SUBCDE = 0x00
ME_CALIBRATION_GETCAL_SUBCDE = 0x01
ME_SAVE_DCD_PERIODIC_ENABLE_SUBCDE = 0x00
ME_SAVE_DCD_PERIODIC_DISABLE_SUBCDE = 0x00

#Reports Summary depending on BNO device 
BNO_REPORT_ACCELEROMETER = 0x01
BNO_REPORT_GYROSCOPE = 0x02
BNO_REPORT_MAGNETOMETER = 0x03
BNO_REPORT_LINEAR_ACCELERATION = 0x04
BNO_REPORT_ROTATION_VECTOR = 0x05
BNO_REPORT_GRAVITY = 0x06
BNO_REPORT_UNCALIBRATED_GYROSCOPE = 0x07
BNO_REPORT_GAME_ROTATION_VECTOR = 0x08
BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR = 0x09
BNO_REPORT_PRESSURE = 0x0A
BNO_REPORT_AMBIENT_LIGHT = 0x0B
BNO_REPORT_HUMIDITY = 0x0C
BNO_REPORT_PROXIMITY = 0x0D
BNO_REPORT_TEMPERATURE = 0x0E
BNO_REPORT_UNCALIBRATED_MAGNETOMETER = 0x0F
BNO_REPORT_TAP_DETECTOR = 0x10
BNO_REPORT_STEP_COUNTER = 0x11
BNO_REPORT_SIGNIFICANT_MOTION = 0x12
BNO_REPORT_STABILITY_CLASSIFIER = 0x13
BNO_REPORT_RAW_ACCELEROMETER = 0x14
BNO_REPORT_RAW_GYROSCOPE = 0x15
BNO_REPORT_RAW_MAGNETOMETER = 0x16
BNO_REPORT_SAR = 0x17
BNO_REPORT_STEP_DETECTOR = 0x18
BNO_REPORT_SHAKE_DETECTOR = 0x19
BNO_REPORT_FLIP_DETECTOR = 0x1A
BNO_REPORT_PICKUP_DETECTOR = 0x1B
BNO_REPORT_STABILITY_DETECTOR = 0x1C
BNO_REPORT_ACTIVITY_CLASSIFIER = 0x1E
BNO_REPORT_SLEEP_DETECTOR = 0x1F
BNO_REPORT_TILT_DETECTOR = 0x20
BNO_REPORT_POCKET_DETECTOR = 0x21
BNO_REPORT_CIRCLE_DETECTOR = 0x22
BNO_REPORT_HEART_RATE_MONITOR = 0x23
BNO_REPORT_ARVR_STABILIZED_ROTATION_VECTOR = 0x28
BNO_REPORT_ARVR_STABILIZED_GAME_ROTATION_VECTOR = 0x29
BNO_REPORT_GYRO_INTEGRATED_ROTATION_VECTOR = 0x2A

#Timeouts (ms) and intervals (us)
DEFAULT_REPORT_INTERVAL = 50000
QUAT_READ_TIMEOUT = 500
PACKET_READ_TIMEOUT = 2000 
FEATURE_ENABLE_TIMEOUT = 2000
DEFAULT_TIMEOUT = 2000

#Quaternions and precisions
QUAT_Q_POINT = 0x05 # 14 by default
Q_POINT_14_SCALAR = 2 ** (14 * -1)
Q_POINT_12_SCALAR = 2 ** (12 * -1)
Q_POINT_10_SCALAR = 2 ** (10 * -1)
Q_POINT_9_SCALAR = 2 ** (9 * -1)
Q_POINT_8_SCALAR = 2 ** (8 * -1)
Q_POINT_4_SCALAR = 2 ** (4 * -1)
GYRO_SCALAR = Q_POINT_9_SCALAR
ACCEL_SCALAR = Q_POINT_8_SCALAR
QUAT_SCALAR = Q_POINT_14_SCALAR
GEOQUAT_SCALAR = Q_POINT_12_SCALAR
MAG_SCALAR = Q_POINT_4_SCALAR

#Report Lengths
REPORT_LENGTHS = {
    SHTP_REPORT_ID_RESPONSE: 16,
    GET_FEATURE_RESPONSE: 17,
    COMMAND_RESPONSE: 16,
    SHTP_REPORT_ID_RESPONSE: 16,
    BASE_TIMESTAMP: 5,
    REBASE_TIMESTAMP: 5,
}

#Raw reports requiring their counterpart to be enabled
RAW_REPORTS = {
    BNO_REPORT_RAW_ACCELEROMETER: BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_RAW_GYROSCOPE: BNO_REPORT_GYROSCOPE,
    BNO_REPORT_RAW_MAGNETOMETER: BNO_REPORT_MAGNETOMETER,
}

#Available sensor reports
AVAIL_SENSOR_REPORTS = {
    BNO_REPORT_ACCELEROMETER: (Q_POINT_8_SCALAR, 3, 10),
    BNO_REPORT_GYROSCOPE: (Q_POINT_9_SCALAR, 3, 10),
    BNO_REPORT_MAGNETOMETER: (Q_POINT_4_SCALAR, 3, 10),
    BNO_REPORT_LINEAR_ACCELERATION: (Q_POINT_8_SCALAR, 3, 10), 
    BNO_REPORT_ROTATION_VECTOR: (Q_POINT_14_SCALAR, 4, 14),
    BNO_REPORT_GRAVITY: (Q_POINT_8_SCALAR, 3, 10),
    BNO_REPORT_GAME_ROTATION_VECTOR: (Q_POINT_14_SCALAR, 4, 12),
    BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR: (Q_POINT_12_SCALAR, 4, 14),
    BNO_REPORT_PRESSURE : (1, 1, 8),
    BNO_REPORT_AMBIENT_LIGHT : (1, 1, 8),
    BNO_REPORT_HUMIDITY  : (1, 1, 6),
    BNO_REPORT_PROXIMITY : (1, 1, 6),
    BNO_REPORT_TEMPERATURE  : (1, 1, 6),
    BNO_REPORT_STEP_COUNTER: (1, 1, 12),
    BNO_REPORT_SHAKE_DETECTOR: (1, 1, 6),
    BNO_REPORT_STABILITY_CLASSIFIER: (1, 1, 6),
    BNO_REPORT_ACTIVITY_CLASSIFIER: (1, 1, 16),
    BNO_REPORT_RAW_ACCELEROMETER: (1, 3, 16),
    BNO_REPORT_RAW_GYROSCOPE: (1, 3, 16),
    BNO_REPORT_RAW_MAGNETOMETER: (1, 3, 16),
}

#Initial reports config
INITIAL_REPORTS = {
    BNO_REPORT_ACTIVITY_CLASSIFIER: {
        "Tilting": -1,
        "most_likely": "Unknown",
        "OnStairs": -1,
        "On-Foot": -1,
        "Other": -1,
        "On-Bicycle": -1,
        "Still": -1,
        "Walking": -1,
        "Unknown": -1,
        "Running": -1,
        "In-Vehicle": -1,
    },
    BNO_REPORT_STABILITY_CLASSIFIER: "Unknown",
    BNO_REPORT_ROTATION_VECTOR: (0.0, 0.0, 0.0, 0.0),
    BNO_REPORT_GAME_ROTATION_VECTOR: (0.0, 0.0, 0.0, 0.0),
    BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR: (0.0, 0.0, 0.0, 0.0),
}

#Dictionnaries for debugging 
CHANNELS_DICTIONARY = {
    0x0: "SHTP_COMMAND",
    0x1: "EXE",
    0x2: "CONTROL",
    0x3: "INPUT_SENSOR_REPORTS",
    0x4: "WAKE_INPUT_SENSOR_REPORTS",
    0x5: "GYRO_ROTATION_VECTOR",
}

REPORTS_DICTIONARY = {
    0x01: "ACCELEROMETER",
    0x02: "GYROSCOPE",
    0x03: "MAGNETIC_FIELD",
    0x04: "LINEAR_ACCELERATION",
    0x05: "ROTATION_VECTOR",
    0x06: "GRAVITY",
    0x07: "UNCALIBRATED_GYROSCOPE",
    0x08: "GAME_ROTATION_VECTOR",
    0x09: "GEOMAGNETIC_ROTATION_VECTOR",
    0x0A: "PRESSURE",
    0x0B: "AMBIENT LIGHT",
    0x0C: "HUMIDITY",
    0x0D: "PROXIMITY",
    0x0E: "TEMPERATURE",
    0x0F: "UNCALIBRATED_MAGNETIC_FIELD",
    0x10: "TAP_DETECTOR",
    0x11: "STEP_COUNTER",
    0x12: "SIGNIFICANT_MOTION",
    0x13: "STABILITY_CLASSIFIER",    
    0x14: "RAW_ACCELEROMETER",
    0x15: "RAW_GYROSCOPE",
    0x16: "RAW_MAGNETOMETER",
    0x17: "SAR",
    0x18: "STEP_DETECTOR",
    0x19: "SHAKE_DETECTOR",
    0x1A: "FLIP_DETECTOR",
    0x1B: "PICKUP_DETECTOR",
    0x1C: "STABILITY_DETECTOR",
    0x1E: "PERSONAL_ACTIVITY_CLASSIFIER",
    0x1F: "SLEEP_DETECTOR",
    0x20: "TILT_DETECTOR",
    0x21: "POCKET_DETECTOR",
    0x22: "CIRCLE_DETECTOR",
    0x23: "HR MONITOR",
    0x28: "ARVR_STABILIZED_ROTATION_VECTOR",
    0x29: "ARVR_STABILIZED_GAME_ROTATION_VECTOR",
    0x2A: "GYRO INTEGRATED ROTATION VECTOR",
    0xF1: "COMMAND_RESPONSE",
    0xF2: "COMMAND_REQUEST",
    0xF3: "FRS_READ_RESPONSE",
    0xF4: "FRS_READ_REQUEST",
    0xF5: "FRS_WRITE_RESPONSE",
    0xF6: "FRS_WRITE_DATA",
    0xF7: "FRS_WRITE_REQUEST",
    0xF8: "PRODUCT_ID_RESPONSE",
    0xF9: "PRODUCT_ID_REQUEST",
    0xFA: "TIMESTAMP_REBASE",
    0xFB: "BASE_TIMESTAMP",
    0xFC: "GET_FEATURE_RESPONSE",
    0xFD: "SET_FEATURE_COMMAND",
    0xFE: "GET_FEATURE_REQUEST",
}

ACTITITIES = [ "Unknown", "In-Vehicle", "On-Bicycle", "On-Foot", "Still",
        "Tilting", "Walking", "Running", "OnStairs"]


ENABLED_ACTIVITIES = (
    0x1FF  # All activities; 1 bit set for each of 8 activities, + Unknown
)

PacketHeader = namedtuple(
    "PacketHeader",
    [
        "channel_number",
        "sequence_number",
        "data_length",
        "packet_byte_count",
    ],
)

REPORT_ACCURACY_STATUS = [
    "Accuracy Unreliable",
    "Low Accuracy",
    "Medium Accuracy",
    "High Accuracy",
]

############ PACKET PARSING GENERAL FUNCTIONS ###########################

def _insertCOMMAND_REquest_report(command, buffer, next_sequence_number, command_params=None):
    if command_params and len(command_params) > 9:
        raise AttributeError(
            "Command request reports can only have up to 9 arguments but %d were given"
            % len(command_params)
        )
    for _i in range(12):
        buffer[_i] = 0
    buffer[0] = COMMAND_REQUEST
    buffer[1] = next_sequence_number
    buffer[2] = command
    if command_params is None:
        return

    for idx, param in enumerate(command_params):
        buffer[3 + idx] = param

#Class representing a Hillcrest LaboratorySensor Hub Transport packet Header ONLY
class Header:
    
    def __init__(self, packet_header_bytes, debug=False):
        self._debug = debug
        self.header = self.header_from_buffer(packet_header_bytes)
        
    def __str__(self):
        length = self.header.packet_byte_count
        outstr = "\t\t\t\tHeader:\n"
        outstr += "\t\t\t\t\tChan Num:\t\t%d (%s)\n" % (
            self.header.channel_number,
            CHANNELS_DICTIONARY[self.header.channel_number],
        )
        outstr += "\t\t\t\t\tSequ Num:\t\t%d\n" % self.header.sequence_number
        outstr += "\t\t\t\t\tData Len:\t\t%d\n" % self.header.data_length
        outstr += "\t\t\t\t\tPack Cnt:\t\t%d" % self.header.packet_byte_count
        return outstr
        
    @classmethod
    def header_from_buffer(cls, header_bytes):
        #Creates a `Header` object from a given buffer
        header_byte_count, channel_number, sequence_number = unpack_from("<HBB", header_bytes)
        header_byte_count &= ~0x8000
        data_length = max(0, header_byte_count - 4)
        return PacketHeader(channel_number, sequence_number, data_length, header_byte_count)

class Packet:
    #A class representing a Hillcrest LaboratorySensor Hub Transport packet

    def __init__(self, packet_bytes, debug=False):
        self._debug = debug
        self.header = self.header_from_buffer(packet_bytes)
        data_end_index = self.header.data_length + 4
        self.data = packet_bytes[4:data_end_index]

    def __str__(self):
        length = self.header.packet_byte_count
        outstr = "\t\t\t\tPacket Header:\n"
        outstr += "\t\t\t\t\tData Len:\t\t%d\n" % (self.header.data_length)
        outstr += "\t\t\t\t\tChan Num:\t\t%d (%s)\n" % (
            self.channel_number,
            CHANNELS_DICTIONARY[self.channel_number],
        )
        if self.channel_number in [
            BNO_CHANNEL_CONTROL,
            BNO_CHANNEL_INPUT_SENSOR_REPORTS,
        ]:
            if self.report_id in REPORTS_DICTIONARY:
                outstr += "\t\t\t\t\tRepo Typ:\t\t0x%x (%s)\n" % (
                    self.report_id,
                    REPORTS_DICTIONARY[self.report_id], 
                )
            else:
                outstr += "\t\t\t\t\t**UNKNOWN Report Type **: %s\n" % hex(
                    self.report_id
                )

            if (
                self.report_id > 0xF0
                and len(self.data) >= 6
                and self.data[5] in REPORTS_DICTIONARY
            ):
                outstr += "\t\t\t\t\tSen Type:\t\t%s(%s)\n" % (
                    hex(self.data[5]),
                    REPORTS_DICTIONARY[self.data[5]],
                )

            if (
                self.report_id == 0xFC
                and len(self.data) >= 6
                and self.data[1] in REPORTS_DICTIONARY
            ):
                outstr += "\t\t\t\t\tEnabled Feature: %s(%s)\n" % (
                    REPORTS_DICTIONARY[self.data[1]],
                    hex(self.data[5]),
                )
        outstr += "\t\t\t\t\tSequ Num:\t\t%s\n" % self.header.sequence_number
        outstr += "\t\t\t\tPacket Data:"

        for idx, packet_byte in enumerate(self.data[:length]):
            packet_index = idx + 4
            if (packet_index % 4) == 0:
                outstr += "\n\t\t\t\t\t[0x{:02X}]\t".format(packet_index)
            outstr += "0x{:02X} ".format(packet_byte)

        return outstr

    @property
    def report_id(self):
        #The Packet's Report ID
        return self.data[0]

    @property
    def channel_number(self):
        #The packet channel
        return self.header.channel_number

    @classmethod
    def header_from_buffer(cls, packet_bytes):
        #Creates a `PacketHeader` object from a given buffer
        packet_byte_count, channel_number, sequence_number = unpack_from("<HBB", packet_bytes)
        packet_byte_count &= ~0x8000
        data_length = max(0, packet_byte_count - 4)
        return PacketHeader(channel_number, sequence_number, data_length, packet_byte_count)

    @classmethod
    def is_error(cls, header):
        #Returns True if the header is an error condition

        if header.channel_number > 5:
            return True
        if header.packet_byte_count == 0xFFFF and header.sequence_number == 0xFF:
            return True
        return False
    
    def _dbg(self, *args, **kwargs):
        if self._debug:
            print("DBG:\t",LIBNAME,":\t", *args, **kwargs)

class PacketError(Exception):
    """Raised when the packet couldnt be parsed"""
    pass

class BNO08X:
    #Library for the BNO08x IMUs from Hillcrest Laboratories

    def __init__(self, i2c, address=None, rst_pin=None, int_pin=None, int_handler=None, debug=False):
        
        self._debug = debug
        self._i2c = i2c
        self._reset_pin = rst_pin
        self._ready = False
        
        #Searching for BNO08x adresses on I2C bus if not specifyed
        if address is None :
            devices = set(self._i2c.scan())
            mpus = devices.intersection(set(BNO08X_DEFAULT_ADDRESS))
            nb_of_mpus = len(mpus)
            if nb_of_mpus == 0:
                raise ValueError("No BNO08x detected")
            elif nb_of_mpus == 1:
                self._bno_add = mpus.pop()
                self._dbg("DEVICE FOUND AT ADDRESS... ",hex(self._bno_add))
                self._ready = True
            else:
                raise ValueError("Two BNO08x detected: must specify a device address")
        else :
            self._bno_add = address
            
        if int_pin is not None:
            self.int_pin = int_pin
            self.int_handler = int_handler
            self.int_locked = False
            int_pin.irq(trigger=int_pin.IRQ_FALLING | int_pin.IRQ_RISING,
                        handler=self.int_handle)
        
        self._dbg("INITIALISATION...")
        self._data_buffer = bytearray(DATA_BUFFER_SIZE)
        self._data_buffer_mv = memoryview(self._data_buffer)
        self._command_buffer = bytearray(12)
        self._packet_slices = []

        # TODO: this is wrong there should be one per channel per direction
        self._sequence_number = [0, 0, 0, 0, 0, 0]
        self._two_ended_sequence_numbers = {
            "send": {},  # holds the next seq number to send with the report id as a key
            "receive": {},
        }
        self._dcd_saved_at = -1
        self._me_calibration_started_at = -1
        self._calibration_complete = False
        self._ME_TARE_CDE_started_at = -1
        self._tare_complete = False
        self._magnetometer_accuracy = 0
        self._wait_for_initialize = True
        self._init_complete = False
        self._id_read = False		#Initialisation we do not know id
        # for saving the most recent reading when decoding several packets
        self._readings = {}
        self.initialize()

    def initialize(self):
        #Initialize the sensor
        for _ in range(3):
            self.hard_reset()
            self.soft_reset()
            try:
                if self._check_id():
                    break
            except:  # pylint:disable=bare-except
                sleep_ms(500)
        else:
            raise RuntimeError("Could not read ID")
        
    def int_handle(self, pin):
        if not pin.value() and not self.int_locked:
            self.int_locked = True  # Lock Interrupt
            buff = "New BNO Message"
            #if buff is not None:
            #    self.int_handler(buff)
        elif pin.value() and self.int_locked:
            self.int_locked = False  # Unlock interrupt

    #Reset the sensor to an initial unconfigured state
    def soft_reset(self):
        self._dbg("SOFT RESETTING...")
        data = bytearray(1)
        data[0] = 1
        _seq = self._send_packet(BNO_CHANNEL_EXE, data)
        sleep_ms(500)
        _seq = self._send_packet(BNO_CHANNEL_EXE, data)
        sleep_ms(500)

        for _i in range(3):
            try:
                _packet = self._read_packet()
            except PacketError:
                sleep_ms(500)
        self._dbg("SOFT RESETTING... OK!")

    #Hardware reset the sensor to an initial unconfigured state
    def hard_reset(self):
        
        self._dbg("HARD RESETTING...")
        if self._reset_pin == None :
            return
        from machine import Pin  # pylint:disable=import-outside-toplevel

        self._reset = Pin(self._reset_pin, Pin.OUT)
        self._reset.value(1)
        sleep_ms(10)
        self._reset.value(0)
        sleep_ms(10)
        self._reset.value(1)
        sleep_ms(10)
        
    # Enable a given feature of the BNO08x
    # TODO: add docs for available features
    # TODO2: I think this should call an fn that imports all the bits for the given feature
    # so we're not carrying around  stuff for extra features
    def enable_feature(self, feature_id):
        self._dbg("ENABLING FEATURE ID...", feature_id)
        
        if feature_id == BNO_REPORT_ACTIVITY_CLASSIFIER:
            set_feature_report = self._get_feature_enable_report(feature_id, sensor_specific_config=ENABLED_ACTIVITIES)
        else:
            set_feature_report = self._get_feature_enable_report(feature_id)

        feature_dependency = RAW_REPORTS.get(feature_id, None)
        # if the feature was enabled it will have a key in the readings dict
        if feature_dependency and feature_dependency not in self._readings:
            self._dbg("\tEnabling feature depencency:", feature_dependency)
            self.enable_feature(feature_dependency)

        self._send_packet(BNO_CHANNEL_CONTROL, set_feature_report)
        
        start_time = ticks_ms()
        while  ticks_diff(ticks_ms(), start_time) < FEATURE_ENABLE_TIMEOUT :
            self._process_available_packets(max_packets=10)
            self._dbg("Feature IDs",self._readings)
            if feature_id in self._readings:
                return
        raise RuntimeError("BNO08X_I2C : ENABLING FEATURE ID : Was not able to enable feature", feature_id)
        
    @property
    def ready(self):
        return self._ready

    @property
    def acc(self):
        #A tuple = acceleration measurements on the X, Y, and Z axes in m/s²
        self._process_available_packets()
        try:
            return self._readings[BNO_REPORT_ACCELEROMETER]
        except KeyError:
            raise RuntimeError("No accel report found, is it enabled?") from None
        
    @property
    def acc_raw(self):
        #Returns the sensor's raw, unscaled value from the accelerometer registers
        self._process_available_packets()
        try:
            raw_acceleration = self._readings[BNO_REPORT_RAW_ACCELEROMETER]
            return raw_acceleration
        except KeyError:
            raise RuntimeError(
                "No raw acceleration report found, is it enabled?"
            ) from None
        
    @property
    def acc_linear(self):
        #A tuple = current linear acceleration values on the X, Y, and Z axes in m/s²
        self._process_available_packets()
        try:
            return self._readings[BNO_REPORT_LINEAR_ACCELERATION]
        except KeyError:
            raise RuntimeError("No lin. accel report found, is it enabled?") from None

    @property
    def gyro(self):
        #A tuple = Gyro's rotation measurements on the X, Y, and Z axes in rad/s
        self._process_available_packets()
        try:
            return self._readings[BNO_REPORT_GYROSCOPE]
        except KeyError:
            raise RuntimeError("No gyro report found, is it enabled?") from None

    @property
    def gyro_raw(self):
        #Returns the sensor's raw, unscaled value from the gyro registers
        self._process_available_packets()
        try:
            raw_gyro = self._readings[BNO_REPORT_RAW_GYROSCOPE]
            return raw_gyro
        except KeyError:
            raise RuntimeError("No raw gyro report found, is it enabled?") from None

    @property
    def mag(self):
        #A tuple of the current magnetic field measurements on the X, Y, and Z axes
        self._process_available_packets()  # decorator?
        try:
            return self._readings[BNO_REPORT_MAGNETOMETER]
        except KeyError:
            raise RuntimeError("No magfield report found, is it enabled?") from None

    @property
    def mag_raw(self):
        #Returns the sensor's raw, unscaled value from the magnetometer registers
        self._process_available_packets()
        try:
            raw_magnetic = self._readings[BNO_REPORT_RAW_MAGNETOMETER]
            return raw_magnetic
        except KeyError:
            raise RuntimeError("No raw magnetic report found, is it enabled?") from None

    @property
    def quaternion(self):
        #A quaternion representing the current rotation vector
        self._process_available_packets()
        try:
            return self._readings[BNO_REPORT_ROTATION_VECTOR]
        except KeyError:
            raise RuntimeError("No quaternion report found, is it enabled?") from None

    @property
    def euler(self):
        #A 3-tupple representing the current Pan Tilt and Roll euler angle in degree
        self._process_available_packets()
        try:
            q = self._readings[BNO_REPORT_ROTATION_VECTOR]
        except KeyError:
            raise RuntimeError("No quaternion report found, is it enabled?") from None
        
        jsqr = q[1] * q[1]
        t0 = +2.0 * (q[3] * q[0] + q[1] * q[2])
        t1 = +1.0 - 2.0 * (q[0] * q[0] + jsqr)
        Roll = degrees(atan2(t0, t1))

        t2 = +2.0 * (q[3] * q[1] - q[2] * q[0])
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Tilt = degrees(asin(t2))

        t3 = +2.0 * (q[3] * q[2] + q[0] * q[1])
        t4 = +1.0 - 2.0 * (jsqr + q[2] * q[2])
        Pan = degrees(atan2(t3, t4))

        return (Roll, Tilt, Pan)

    @property
    def geomagnetic_quat(self):
        #A quaternion representing the current geomagnetic rotation vector
        self._process_available_packets()
        try:
            return self._readings[BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR]
        except KeyError:
            raise RuntimeError(
                "No geomag quaternion report found, is it enabled?"
            ) from None

    @property
    def game_quat(self):
        """A quaternion representing the current rotation vector expressed as a quaternion with no
        specific reference for heading, while roll and pitch are referenced against gravity. To
        prevent sudden jumps in heading due to corrections, the `game_quaternion` property is not
        corrected using the magnetometer. Some drift is expected"""
        self._process_available_packets()
        try:
            return self._readings[BNO_REPORT_GAME_ROTATION_VECTOR]
        except KeyError:
            raise RuntimeError(
                "No game quaternion report found, is it enabled?"
            ) from None

    @property
    def steps(self):
        """The number of steps detected since the sensor was initialized"""
        self._process_available_packets()
        try:
            return self._readings[BNO_REPORT_STEP_COUNTER]
        except KeyError:
            raise RuntimeError("No steps report found, is it enabled?") from None

    @property
    def gravity(self):
        """A tuple representing the gravity vector in the X, Y, and Z components
        axes in meters per second squared"""
        self._process_available_packets()
        try:
            return self._readings[BNO_REPORT_GRAVITY]
        except KeyError:
            raise RuntimeError("No gravity report found, is it enabled?") from None

    @property
    def shake(self):
        """True if a shake was detected on any axis since the last time it was checked
        This property has a "latching" behavior where once a shake is detected, it will stay in a
        "shaken" state until the value is read. This prevents missing shake events but means that
        this property is not guaranteed to reflect the shake state at the moment it is read
        """
        self._process_available_packets()
        try:
            shake_detected = self._readings[BNO_REPORT_SHAKE_DETECTOR]
            # clear on read
            if shake_detected:
                self._readings[BNO_REPORT_SHAKE_DETECTOR] = False
            return shake_detected
        except KeyError:
            raise RuntimeError("No shake report found, is it enabled?") from None

    @property
    def stability_classif(self):
        """Returns the sensor's assessment of it's current stability, one of:
        * "Unknown" - The sensor is unable to classify the current stability
        * "On Table" - The sensor is at rest on a stable surface with very little vibration
        * "Stationary" -  The sensor’s motion is below the stable threshold but\
        the stable duration requirement has not been met. This output is only available when\
        gyro calibration is enabled
        * "Stable" - The sensor’s motion has met the stable threshold and duration requirements.
        * "In motion" - The sensor is moving.
        """
        self._process_available_packets()
        try:
            stability_classification = self._readings[BNO_REPORT_STABILITY_CLASSIFIER]
            return stability_classification
        except KeyError:
            raise RuntimeError(
                "No stability classification report found, is it enabled?"
            ) from None

    @property
    def activity_classif(self):
        """Returns the sensor's assessment of the activity that is creating the motions\
        that it is sensing, one of:
        * "Unknown"
        * "In-Vehicle"
        * "On-Bicycle"
        * "On-Foot"
        * "Still"
        * "Tilting"
        * "Walking"
        * "Running"
        * "On Stairs"
        """
        self._process_available_packets()
        try:
            activity_classification = self._readings[BNO_REPORT_ACTIVITY_CLASSIFIER]
            return activity_classification
        except KeyError:
            raise RuntimeError(
                "No activity classification report found, is it enabled?"
            ) from None

    def tare(self, axis = 7, outputs = 2):
        #Tare the sensor
        self._dbg("MOTION ENGINE TARE BEING DONE...")
        self._send_ME_cde(ME_TARE_CDE,
            [
                0,					#Perform Tare Now
                axis,				#Perform All axis (7) by default
                outputs, 			#Apply to all motion outputs (2) by default
                0, 0, 0, 0, 0, 0,	#6-11 Reserved
            ]
        )
        self._calibration_complete = True

    def calibration(self):
        #Self-calibrate the sensor
        self._dbg("MOTION ENGINE CALIBRATION BEING DONE...")
        self._send_ME_cde(ME_CALIBRATION_CDE,
            [
                1,  # calibrate accel
                1,  # calibrate gyro
                1,  # calibrate mag
                ME_CALIBRATION_CONFIG_SUBCDE,
                0,  # calibrate planar acceleration
                0,  # 'on_table' calibration
                0,  # reserved
                0,  # reserved
                0,  # reserved
            ]
        )
        self._calibration_complete = True

    @property
    def calibration_status(self):
        #Get the status of the self-calibration
        self._dbg("MOTION ENGINE GETTING CALIBRATION STATUS...")
        self._send_ME_cde(ME_CALIBRATION_CDE,
            [
                0,  # calibrate accel
                0,  # calibrate gyro
                0,  # calibrate mag
                ME_CALIBRATION_GETCAL_SUBCDE,
                0,  # calibrate planar acceleration
                0,  # 'on_table' calibration
                0,  # reserved
                0,  # reserved
                0,  # reserved
            ]
        )
        return self._magnetometer_accuracy
    
    def calibration_save(self):
        #Save the self-calibration data by sending a DCD save command
        local_buffer = bytearray(12)
        _insertCOMMAND_REquest_report(
            ME_SAVE_DCD_CDE,
            local_buffer,  # should use self._data_buffer :\ but send_packet don't
            self._get_report_seq_id(COMMAND_REQUEST),
        )
        self._send_packet(BNO_CHANNEL_CONTROL, local_buffer)
        self._increment_report_seq(COMMAND_REQUEST)
        
        start_time = ticks_ms()
        while ticks_diff(ticks_ms(), start_time) < DEFAULT_TIMEOUT:
            self._process_available_packets()
            if self._dcd_saved_at > start_time:
                return
        raise RuntimeError("Could not save calibration data")
    
    #====== Below are internal function ============

    def _send_ME_cde(self, me_cde, subcommand):
        local_buffer = self._command_buffer
        _insertCOMMAND_REquest_report(
            me_cde,
            self._command_buffer,  # should use self._data_buffer :\ but send_packet don't
            self._get_report_seq_id(COMMAND_REQUEST),
            subcommand,
        )
        self._send_packet(BNO_CHANNEL_CONTROL, local_buffer)
        self._increment_report_seq(COMMAND_REQUEST)
        
    
    def _process_available_packets(self, max_packets=None):
        processed_count = 0
        self._dbg("PROCESSING AVAILABLE PACKETS : Max_packet = ",processed_count,"/",max_packets)
        while self._data_ready:
            if max_packets and processed_count > max_packets:
                return
            # print("reading a packet")
            try:
                new_packet = self._read_packet()
            except PacketError:
                continue
            self._handle_packet(new_packet)
            processed_count += 1
            self._dbg("\t Packets processed = ",processed_count)    
        self._dbg("PROCESSING AVAILABLE PACKETS : DONE!")

    def _wait_for_packet_type(self, channel_number, report_id=None, timeout=10000, debug=True):
        if report_id:
            report_id_str = " with report id %s" % hex(report_id)
        else:
            report_id_str = ""
        self._dbg("WAITING FOR PACKET on channel", channel_number, report_id_str)
        
        start_time = ticks_ms()
        while ticks_diff(ticks_ms(), start_time) < timeout :
            new_packet = self._wait_for_packet()
            self._dbg("NEW PACKET : ")
            if self._debug:
                print(new_packet)
            if new_packet.channel_number == channel_number:
                if report_id:
                    if new_packet.report_id == report_id:
                        return new_packet
                else:
                    return new_packet
            if new_packet.channel_number not in (BNO_CHANNEL_EXE, BNO_CHANNEL_SHTP_COMMAND):
                self._dbg("passing packet to handler for de-slicing")
                self._handle_packet(new_packet)
        raise RuntimeError("Timed out waiting for a packet on channel", channel_number)

    def _wait_for_packet(self, timeout=PACKET_READ_TIMEOUT):
        self._dbg("WAITING FOR PACKET")
        start_time = ticks_ms()
        while  ticks_diff(ticks_ms(), start_time) < timeout :
            if not self._data_ready:
                print("NOT READY")
                continue
            new_packet = self._read_packet()
            return new_packet
        raise RuntimeError("Timed out waiting for a packet")

    # update the cached sequence number so we know what to increment from
    # TODO: this is wrong there should be one per channel per direction
    # and apparently per report as well
    def _update_sequence_number(self, new_packet):
        channel = new_packet.channel_number
        seq = new_packet.header.sequence_number
        self._sequence_number[channel] = seq
        
    def _handle_packet(self, packet):
        # split out reports first
        self._dbg("HANDLING PACKET")
        try:
            # get first report id, loop up its report length, read that many bytes, parse them
            next_byte_index = 0
            while next_byte_index < packet.header.data_length:
                report_id = packet.data[next_byte_index]
                if report_id < 0xF0:  # it's a sensor report
                    required_bytes = AVAIL_SENSOR_REPORTS[report_id][2]
                else :
                    required_bytes = REPORT_LENGTHS[report_id]
                unprocessed_byte_count = packet.header.data_length - next_byte_index
                # handle incomplete remainder
                if unprocessed_byte_count < required_bytes:
                    raise RuntimeError("Unprocessable Batch bytes", unprocessed_byte_count)
                # we have enough bytes to read so add a slice to the list that was passed in
                report_slice = packet.data[next_byte_index : next_byte_index + required_bytes]
                self._packet_slices.append([report_slice[0], report_slice])
                next_byte_index = next_byte_index + required_bytes
            while len(self._packet_slices) > 0:
                self._process_report(*self._packet_slices.pop())    
        except Exception as error:
            self._dbg(packet)
            raise error

    def _handle_control_report(self, report_id, report_bytes):
        self._dbg("CONTROLLING REPORT = ",REPORTS_DICTIONARY[report_id])
        
        if report_id == SHTP_REPORT_ID_RESPONSE:
            report_id, sw_major, sw_minor, sw_part_number, sw_build_number, sw_patch = unpack_from("<HBBIIH", report_bytes)
            if report_id != SHTP_REPORT_ID_RESPONSE:
                raise AttributeError("Wrong report id for sensor id: %s" % hex(buffer[0]))
            self._dbg("\tFROM PACKET SLICE:")
            self._dbg("\t*** Part Number: %d" % sw_part_number)
            self._dbg("\t*** Software Version: %d.%d.%d" % (sw_major, sw_minor, sw_patch))
            self._dbg("\tBuild: %d" % (sw_build_number)) 

        if report_id == GET_FEATURE_RESPONSE:
            # report_id, feature_report_id, feature_flags, change_sensitivity, report_interval
            # batch_interval_word, sensor_specific_configuration_word
            _report_id, feature_report_id, *_remainder = unpack_from("<BBBHIII", report_bytes)
            self._readings[feature_report_id] = INITIAL_REPORTS.get(feature_report_id, (0.0, 0.0, 0.0))
            if self._debug :
                outstr = "\t\t\t\t\tReport Id   \t\t%d" % _report_id
                outstr += "\n\t\t\t\t\tFeat Rep. Id\t\t%d" % feature_report_id
                outstr += "\n\t\t\t\t\tReadings    \t\t%s" % str(self._readings)
                print(outstr)
        if report_id == COMMAND_RESPONSE:
            self._handle_command_response(report_bytes)

    def _handle_command_response(self, report_bytes):
        # CMD response report:
        # 0 Report ID = 0xF1, # 1 Sequence number, # 2 Command, # 3 Command sequence number, # 4 Response sequence number
        # 5 R0-10 A set of response values. The interpretation of these values is specific to the response for each command.
        # status, accel_en, gyro_en, mag_en, planar_en, table_en, *_reserved = response_values
        _report_id, _seq_number, command, _command_seq_number, _response_seq_number, = unpack_from("<BBBBB", report_bytes)
        command_status, *_rest = unpack_from("<BBBBBBBBBBB", report_bytes, 5)

        if command == ME_CALIBRATION_CDE and command_status == 0:
            self._me_calibration_started_at = ticks_ms()

        if command == ME_SAVE_DCD_CDE:
            if command_status == 0:
                self._dcd_saved_at = ticks_ms()
            else:
                raise RuntimeError("Unable to save calibration data")

    def _process_report(self, report_id, report_bytes):
        self._dbg("PROCESSING REPORTS")
        if report_id >= 0xF0:
            self._handle_control_report(report_id, report_bytes)
            return
        
        if self._debug:
            outstr = "\t\t\t\tProcessing %s report" % REPORTS_DICTIONARY[report_id]
            for idx, packet_byte in enumerate(report_bytes):
                packet_index = idx
                if (packet_index % 4) == 0:
                    outstr += "\n\t\t\t\t\t[0x{:02X}] ".format(packet_index)
                outstr += "0x{:02X} ".format(packet_byte)
            print(outstr) 

        if report_id == BNO_REPORT_STEP_COUNTER:
            self._readings[report_id] = unpack_from("<H", report_bytes, 8)          
            return

        if report_id == BNO_REPORT_SHAKE_DETECTOR:
            shake_bitfield = unpack_from("<H", report_bytes, 4)
            shake_detected = (shake_bitfield & 0x111) > 0
            
            # shake not previously detected - auto cleared by 'shake' property
            try:
                if not self._readings[BNO_REPORT_SHAKE_DETECTOR]:
                    self._readings[BNO_REPORT_SHAKE_DETECTOR] = shake_detected
            except KeyError:
                pass
            return

        if report_id == BNO_REPORT_STABILITY_CLASSIFIER:
            classification_bitfield = unpack_from("<B", report_bytes, 4)
            stability_classification = ["Unknown", "On Table", "Stationary", "Stable", "In motion"][classification_bitfield]
            self._readings[BNO_REPORT_STABILITY_CLASSIFIER] = stability_classification
            return

        if report_id == BNO_REPORT_ACTIVITY_CLASSIFIER:        
            # 0 Report ID = 0x1E, # 1 Sequence number, # 2 Status, 3 Delay, 4 Page Number + EOS
            # 5 Most likely state, # 6-15 Classification (10 x Page Number) + confidence
            end_and_page_number, most_likely = unpack_from("<BB", report_bytes, 4)
            # last_page = (end_and_page_number & 0b10000000) > 0
            page_number = end_and_page_number & 0x7F
            confidences = unpack_from("<BBBBBBBBB", report_bytes, 6)
            activity_classification = {}
            activity_classification["most_likely"] = ACTITITIES[most_likely]
            for idx, raw_confidence in enumerate(confidences):
                confidence = (10 * page_number) + raw_confidence
                activity_string = ACTITITIES[idx]
                activity_classification[activity_string] = confidence
            self._readings[BNO_REPORT_ACTIVITY_CLASSIFIER] = activity_classification
            return
        
        #General case, parsing the report data with only 16-bit fields
        data_offset = 4  # this may not always be true
        report_id = report_bytes[0]
        scalar, count, _report_length = AVAIL_SENSOR_REPORTS[report_id]
        if report_id in RAW_REPORTS: # raw reports are unsigned
            format_str = "<H"
        else:
            format_str = "<h"
        results = []
        accuracy = unpack_from("<B", report_bytes, 2)[0]
        accuracy &= 0b11

        for _offset_idx in range(count):
            total_offset = data_offset + (_offset_idx * 2)
            raw_data = unpack_from(format_str, report_bytes, total_offset)[0]
            scaled_data = raw_data * scalar
            results.append(scaled_data)
        sensor_data = tuple(results)
        if self._debug:
            outstr = "\t\t\t\tReading for %s %s Accuracy %d" % (REPORTS_DICTIONARY[report_id], str(sensor_data), accuracy)
            print(outstr)
        if report_id == BNO_REPORT_MAGNETOMETER:
            self._magnetometer_accuracy = accuracy
        # TODO: FIXME; Sensor reports are batched in a LIFO which means that multiple reports
        # for the same type will end with the oldest/last being kept and the other
        # newer reports thrown away
        self._readings[report_id] = sensor_data

    # TODO: Make this a Packet creation
    @staticmethod
    def _get_feature_enable_report(feature_id, report_interval=DEFAULT_REPORT_INTERVAL, sensor_specific_config=0):
        set_feature_report = bytearray(17)
        set_feature_report[0] = SET_FEATURE_COMMAND
        set_feature_report[1] = feature_id
        pack_into("<I", set_feature_report, 5, report_interval)
        pack_into("<I", set_feature_report, 13, sensor_specific_config)
        return set_feature_report

    def _check_id(self):
        self._dbg("CHECKING ID...")
        if self._id_read:
            return True
        data = bytearray(2)
        data[0] = SHTP_REPORT_ID_REQUEST
        data[1] = 0  # padding
        self._send_packet(BNO_CHANNEL_CONTROL, data)
        
        while True:
            self._wait_for_packet_type(
                BNO_CHANNEL_CONTROL, SHTP_REPORT_ID_RESPONSE
            )
            sensor_id = self._parse_sensor_id()
            if sensor_id:
                self._id_read = True
                return True
            self._dbg("CHECKING ID : Packet didn't have sensor ID report, trying again")
        return False

    def _parse_sensor_id(self):
        self._dbg("PARSE SENSOR ID : BUFFER DATA IS : ", self._data_buffer[4])
        if not self._data_buffer[4] == SHTP_REPORT_ID_RESPONSE:
            return None
        sw_major, sw_minor, sw_part_number, sw_build_number, sw_patch = unpack_from("<BBIIH", self._data_buffer, 6)
        self._dbg("\t*** Part Number: %d" % sw_part_number)
        self._dbg("\t*** Software Version: %d.%d.%d" % (sw_major, sw_minor, sw_patch))
        self._dbg("\t*** Build: %d" % (sw_build_number))
        # TODO: this is only one of the numbers!
        return sw_part_number

    @property
    def _data_ready(self):
        #Check if there is available data on the I2C bus
        header = self._read_header()
        if header.channel_number > 5:
            self._dbg("channel number out of range:", header.channel_number)
        if header.packet_byte_count == 0x7FFF:
            print("Byte count is 0x7FFF/0xFFFF; Error?")
            if header.sequence_number == 0xFF:
                print("Sequence number is 0xFF; Error?")
            ready = False
        else:
            ready = header.data_length > 0
        self._dbg("BNO08X_I2C_DATA READY : ", ready)
        return ready

    #Send a packet = header + packet data
    def _send_packet(self, channel, data):

        self._dbg("SENDING PACKET...")
        #Start to make the packet header, 2 bytes for size, 1 byte for channel and 1 byte for sequence
        data_length = len(data)
        write_length = data_length + 4
        pack_into("<H", self._data_buffer, 0, write_length)
        self._data_buffer[2] = channel
        self._data_buffer[3] = self._sequence_number[channel]
        #Then add the data         
        for idx, send_byte in enumerate(data):
            self._data_buffer[4 + idx] = send_byte
        packet = Packet(self._data_buffer)
        
        if self._debug:
            print(packet)
        
        #Send the packet to the I2C bus and increase the sequence number
        self._i2c.writeto(self._bno_add, self._data_buffer[0:write_length])
        self._sequence_number[channel] = (self._sequence_number[channel] + 1) % 256
        
        return self._sequence_number[channel]

    #Read a packet = header + packet data
    def _read_packet(self):
        
        self._dbg("READING PACKET...")
                
        #Header is 4 bytes long (2 bytes size, 1 byte for channel number and 1 byte for sequence number)
        #Buffer is declared in BNO08X class : self._data_buffer = bytearray(512)
        #But here we use the memoryimage from this buffer which is declared in BNO08X class
        #I2C adress is declared in class : self._bno_add
        
        #Begin with reading a header ==> Expecting a header (4 bytes)
        self._i2c.readfrom_into(self._bno_add, self._data_buffer_mv[0:4])
        
        #Decode the packet header and update sequence number
        header = Packet.header_from_buffer(self._data_buffer[0:4])
        packet_byte_count = header.packet_byte_count
        channel_number = header.channel_number
        sequence_number = header.sequence_number
        data_length = header.data_length
        self._sequence_number[channel_number] = sequence_number
        
        if packet_byte_count == 0:
            self._dbg("\tSKIPPING NO PACKETS AVAILABLE IN bno08x_i2c._read_packet")
            raise PacketError("No packet available")

        #Then we read the packet data to the buffer image
        self._i2c.readfrom_into(self._bno_add, self._data_buffer_mv[0:packet_byte_count])

        #Then process the packet 
        new_packet = Packet(self._data_buffer[0:packet_byte_count])
        
        if self._debug:
            print(new_packet)

        self._update_sequence_number(new_packet)

        return new_packet
    
    def _read_header(self):
        
        #Read only a packet Header
        self._dbg("READING HEADER...")
        
        #Reads the first 4 bytes available as a header ==> Expecting a header
        self._i2c.readfrom_into(self._bno_add, self._data_buffer_mv[0:4])
        packet_header = Packet.header_from_buffer(self._data_buffer[0:4])
        header = Header(self._data_buffer[0:4])
        
        if self._debug:
            print(header)
        
        return packet_header

    def _increment_report_seq(self, report_id):
        current = self._two_ended_sequence_numbers.get(report_id, 0)
        self._two_ended_sequence_numbers[report_id] = (current + 1) % 256

    def _get_report_seq_id(self, report_id):
        return self._two_ended_sequence_numbers.get(report_id, 0)
  
    def _dbg(self, *args, **kwargs):
        if self._debug:
            print("DBG:\t",LIBNAME,":\t", *args, **kwargs)