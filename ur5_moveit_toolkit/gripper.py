"""
Robotiq 2F gripper over a USB/RS-485 adapter (Modbus RTU, slave id 9), as used in the
ART-VS setup. Needs `pyserial` and read/write access to the port
(`sudo usermod -aG dialout $USER`, then log out and in again).

    from ur5_moveit_toolkit import RobotiqGripper
    g = RobotiqGripper("/dev/ttyUSB0")
    g.close(); g.open()
"""
import time

import serial

# Pre-computed Modbus RTU frames (including CRC)
_ACTIVATE = b"\x09\x10\x03\xE8\x00\x03\x06\x00\x00\x00\x00\x00\x00\x73\x30"
_READ_STATUS = b"\x09\x03\x07\xD0\x00\x01\x85\xCF"
_CLOSE = b"\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\xFF\xFF\xFF\x42\x29"  # full speed/force
_OPEN = b"\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\x00\xFF\xFF\x72\x19"


class RobotiqGripper:
    def __init__(self, port="/dev/ttyUSB0"):
        self.ser = serial.Serial(port=port, baudrate=115200, timeout=0.2,
                                 parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                                 bytesize=serial.EIGHTBITS)
        self.ser.write(_ACTIVATE)
        time.sleep(0.1)
        self.ser.write(_READ_STATUS)
        time.sleep(0.1)

    def close(self, wait=1.0):
        self.ser.write(_CLOSE)
        time.sleep(wait)

    def open(self, wait=1.0):
        self.ser.write(_OPEN)
        time.sleep(wait)

    def disconnect(self):
        if self.ser.is_open:
            self.ser.close()
