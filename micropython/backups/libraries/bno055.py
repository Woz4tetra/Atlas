"""
Written by Ben Warwick

bno055.py
Version 6/7/2016
=========

A MicroPython library for the BNO055 IMU sensor. This library was adapted from
Adafruit's Arduino library and contains only the essential features of the
sensor.
(buy it here: https://www.adafruit.com/products/2472)

Usage
-----
import pyb
from libraries.bno055 import BNO055

imu = BNO055(2)  # imu on I2C bus 2

while True:
    yaw, pitch, roll = imu.get_euler()
    w, x, y, z = imu.get_quat()
    print(yaw)
    pyb.delay(1)
"""

import math

import pyb

from ucollections import OrderedDict


class BNO055:
    reg = dict(
        VECTOR_ACCELEROMETER=0x08,
        VECTOR_MAGNETOMETER=0x0e,
        VECTOR_GYROSCOPE=0x14,
        VECTOR_EULER=0x1a,
        VECTOR_LINEARACCEL=0x28,
        VECTOR_GRAVITY=0x2e,
        QUATERNION_DATA=0x20,
        TEMPERATURE=0x34,

        CHIP_ID=0x00,
        SYS_TRIGGER=0x3f,
        OPR_MODE=0x3d,
        PAGE_ID=0x07,
        PWR_MODE=0x3e,

        ACCEL_OFFSET_X_LSB_ADDR=0x55,
        CALIB_STAT_ADDR=0x35,

    )

    modes = dict(
        CONFIG=0x00,
        NDOF=0x0c,
    )

    power_modes = dict(
        NORMAL=0x00,
        LOW=0x01,
        SUSPEND=0x02
    )

    BNO055_ID = 0xa0
    NUM_BNO055_OFFSET_REGISTERS = 22

    def __init__(self, bus, reset_pin=None, default_address=True, declination=(0, 0)):
        self.i2c = pyb.I2C(bus, pyb.I2C.MASTER)
        if reset_pin is not None:
            self.reset_pin = pyb.Pin(reset_pin, pyb.Pin.OUT_PP)
        else:
            self.reset_pin = reset_pin

        if default_address:
            self.address = 0x28
        else:
            self.address = 0x29

        self.declination = \
            (declination[0] + declination[1] / 60) * math.pi / 180

        self.quat_scale = 1.0 / (1 << 14)
        self.sample_delay = 100

        self.default_mode = self.modes['NDOF']

        self.offsets = OrderedDict(
            accel_offset_x=0,
            accel_offset_y=0,
            accel_offset_z=0,

            mag_offset_x=0,
            mag_offset_y=0,
            mag_offset_z=0,

            gyro_offset_x=0,
            gyro_offset_y=0,
            gyro_offset_z=0,

            accel_radius=0,
            mag_radius=0
        )
        self.init_sensor()

    def init_sensor(self):
        pyb.delay(1000)

        addresses = self.i2c.scan()
        if self.address not in addresses:
            raise Exception("Address %s not found during scan: %s" % (
                self.address, addresses))

        if not self.i2c.is_ready(self.address):
            raise Exception("Device not ready")

        pyb.delay(50)
        chip_id = self.read_8(self.reg['CHIP_ID'])
        if ord(chip_id) != self.BNO055_ID:
            pyb.delay(1000)  # wait for boot
            chip_id = self.read_8(self.reg['CHIP_ID'])
            if ord(chip_id) != self.BNO055_ID:
                raise Exception("Chip ID invalid:", chip_id)

        self.set_mode(self.modes['CONFIG'])

        self.write_8(self.reg['SYS_TRIGGER'], 0x20)  # reset
        pyb.delay(1000)
        # while ord(self.read_8(self.reg['CHIP_ID'])) != self.BNO055_ID:
        #     pyb.delay(10)
        self.write_8(self.reg['PWR_MODE'], self.power_modes['NORMAL'])
        pyb.delay(10)

        self.write_8(self.reg['PAGE_ID'], 0)

        self.write_8(self.reg['SYS_TRIGGER'], 0x0)
        pyb.delay(10)

        self.set_mode(self.default_mode)
        pyb.delay(20)

        pyb.delay(100)

        self.set_ext_crystal_use()

        pyb.delay(100)

    def reset(self):
        if self.reset_pin is not None:
            self.reset_pin.low()
            pyb.delay(1)
            self.reset_pin.high()
            self.init_sensor()
        else:
            print("No reset pin defined. BNO055 not reset")

    def set_mode(self, mode):
        self.write_8(self.reg['OPR_MODE'], mode)
        pyb.delay(30)

    def set_ext_crystal_use(self):
        self.set_mode(self.modes['CONFIG'])
        pyb.delay(25)

        self.write_8(self.reg['PAGE_ID'], 0)
        self.write_8(self.reg['SYS_TRIGGER'], 0x80)
        pyb.delay(10)

        self.set_mode(self.default_mode)
        pyb.delay(20)

    def get_lin_accel(self):  # acceleration in m/s^2 (excluding gravity)
        x, y, z = self.get_vector('VECTOR_LINEARACCEL')
        return x / 100.0, y / 100.0, z / 100.0

    def get_gyro(self):  # angular velocity in radians per second
        x, y, z = self.get_vector('VECTOR_GYROSCOPE')
        return x / 900.0, y / 900.0, z / 900.0

    def get_quat(self):
        # quaternion vector (see: https://en.wikipedia.org/wiki/Quaternion)

        buf = self.read_len(self.reg['QUATERNION_DATA'], 8)
        w = (buf[1] << 8) | buf[0]
        x = (buf[3] << 8) | buf[2]
        y = (buf[5] << 8) | buf[4]
        z = (buf[7] << 8) | buf[6]
        return (w * self.quat_scale,
                x * self.quat_scale,
                y * self.quat_scale,
                z * self.quat_scale)

    def get_euler(self):  # yaw, pitch, roll in degrees
        z, y, x = self.get_vector('VECTOR_EULER')
        return z / 16.0, y / 16.0, x / 16.0

    def get_temp(self):  # get temperature in degrees celsius
        return ord(self.read_8(self.reg['TEMPERATURE']))

    def get_accel(self):  # acceleration in m/s^2 (including gravity)
        x, y, z = self.get_vector('VECTOR_ACCELEROMETER')
        return x / 100.0, y / 100.0, z / 100.0

    def get_grav(self):  # gravity vector in m/s^2
        x, y, z = self.get_vector('VECTOR_GRAVITY')
        return x / 100.0, y / 100.0, z / 100.0

    def get_mag(self):  # magnetic field strength in micro-Teslas
        x, y, z = self.get_vector('VECTOR_MAGNETOMETER')
        return x / 16.0, y / 16.0, z / 16.0

    def get_vector(self, vector_type):  # get an x, y, z data array from I2C
        buf = self.read_len(self.reg[vector_type], 6)
        data = []
        for index in range(0, len(buf), 2):
            datum = (buf[index + 1] << 8) | buf[index]
            if datum > 0x7fff:
                datum -= 0x10000
            data.append(datum)
        return data

    def get_calibration(self):
        calib_status = ord(self.read_8(self.reg['CALIB_STAT_ADDR']))
        system = (calib_status >> 6) & 0x03
        gyro = (calib_status >> 4) & 0x03
        accel = (calib_status >> 2) & 0x03
        mag = calib_status & 0x03

        return system, gyro, accel, mag

    def is_fully_calibrated(self):
        for status in self.get_calibration()[1:]:
            if status < 3:
                return False
        return True

    def update_offsets(self):
        if self.is_fully_calibrated():
            self.set_mode(self.modes['CONFIG'])

            calib_data = self.read_len(self.reg['ACCEL_OFFSET_X_LSB_ADDR'],
                                       self.NUM_BNO055_OFFSET_REGISTERS)

            self.set_mode(self.default_mode)

            index = 0
            for offset in self.offsets.keys():
                self.offsets[offset] = (calib_data[index + 1] << 8) | calib_data[index]
                index += 2
#            print(self.offsets)
            return True
        else:
            return False

    def get_heading(self):
        # compass heading in radians (be sure to get the correct declination)

        x, y, z = self.get_mag()
        heading = math.atan2(y, x)
        heading += self.declination
        # Correct for reversed heading
        if heading < 0:
            heading += 2 * math.pi
        # Check for wrap and compensate
        elif heading > 2 * math.pi:
            heading -= 2 * math.pi
        return heading

    def write_8(self, register, data):
        return self.i2c.mem_write(data, self.address, register)

    def read_8(self, register):
        return self.i2c.mem_read(1, self.address, register)

    def read_len(self, register, length):
        #        try:
        return self.i2c.mem_read(length, self.address, register)

# except OSError:
#            return b''
