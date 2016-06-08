"""
Written by Ben Warwick

lidar_lite_v2.py
Version 6/7/2016
=========

A MicroPython library for the lidar_lite_v2 sensor. This library was adapted
from PulseLight3D's Arduino library and contains only the essential features of
the sensor.
(buy it here: http://www.robotshop.com/en/lidar-lite-2-laser-rangefinder-pulsedlight.html)

Usage
-----
import pyb
from libraries.lidar_lite_v2 import LIDARLite

lidar = LIDARLite(2)  # imu on I2C bus 2

while True:
    print(lidar.distance(), lidar.velocity())
    pyb.delay(1)
"""

import pyb

class LIDARLite:
    def __init__(self, bus, config=0, i2c_address=0x62):
        self.address = i2c_address
        self.i2c = pyb.I2C(bus, pyb.I2C.MASTER)

        addresses = self.i2c.scan()

        if self.address not in addresses:
            raise Exception("Address not found during scan: " + str(addresses))

        if not self.i2c.is_ready(self.address):
            raise Exception("Device not ready")
        # pyb.delay(50)

        if config == 0:
            # default configuration
            self.write_8(0x00, 0x00)

        elif config == 1:
            # Set aquisition count to 1/3 default value, faster reads, slightly
            # noisier values
            self.write_8(0x04, 0x00)

        elif config == 2:
            # Low noise, low sensitivity: Pulls decision criteria higher
            # above the noise, allows fewer false detections, reduces
            # sensitivity
            self.write_8(0x1c, 0x20)

        elif config == 3:
            # High noise, high sensitivity: Pulls decision criteria into the
            # noise, allows more false detections, increases sensitivity
            self.write_8(0x1c, 0x60)

        else:
            raise ValueError("Invalid configuration type: ", config)

        pyb.delay(50)

    def distance(self):
        # Take acquisition & correlation processing with DC correction
        self.write_8(0x00, 0x04)

        data = self.read_16(0x8f)
        return (data[0] << 8) + data[1]

    def velocity(self):
        # Write 0xa0 to 0x04 to switch on velocity mode
        self.write_8(0x04, 0xa0)

        # Write 0x04 to register 0x00 to start getting distance readings
        self.write_8(0x00, 0x04)

        return ord(self.read_8(0x09))

    def write_8(self, register, data):
        self.i2c.mem_write(data, self.address, register)

    def read_8(self, register):
        self.i2c.send(register, self.address)
        return self.i2c.recv(1, self.address)

    def read_16(self, register):
        # return self.i2c.mem_read(2, self.address, register)
        self.i2c.send(register, self.address)
        return self.i2c.recv(2, self.address)
