"""
Suction cup gripper control over the XIAO C3's serial protocol.

Sends "GRIP\n" / "RELEASE\n" as UTF-8 at 115200 baud
(8 data bits, no parity, 1 stop bit, no flow control).
"""

import serial
from time import sleep

PORT = "COM9"
BAUDRATE = 115200


class Suction:
    def __init__(self, port=PORT, baudrate=BAUDRATE):
        self._serial = serial.Serial(port, baudrate, timeout=0)
        sleep(2)  # let the controller reset after opening the serial port

    def grip(self):
        self._serial.write(b"GRIP\n")

    def release(self):
        self._serial.write(b"RELEASE\n")

    def status(self):
        self._serial.write(b"STATUS\n")

    def close(self):
        self._serial.close()
