"""
Written by Ben Warwick

pca9685.py
Version 6/7/2016
=========

A MicroPython library for the PCA9685 I2C servo controller. This library was
adapted from Adafruit's Arduino library and contains only the essential features
of the sensor.
(buy it here: https://www.adafruit.com/products/815)

Usage
-----
import pyb
from libraries.pca9685 import PCA9685

servos = PCA9685(2)  # imu on I2C bus 2

for position range(-90, 91):  # servos have a range of -90 to 90
    servos.set_servo(position)  # 0...15 (16 available servos)
    pyb.delay(50)
"""

from math import floor

import pyb


class PCA9685:
    register = dict(
        LED0=0x06,
        MODE1=0x00,
        PRESCALE=0xfe
    )
    address = 0x40
    servo_min = 150
    servo_max = 600

    def __init__(self, bus, pwm_freq=60):
        self.i2c = pyb.I2C(bus, pyb.I2C.MASTER)
        self.reset()
        self.set_pwm_freq(pwm_freq)

    def set_pwm_freq(self, freq):
        freq *= 0.9
        prescale_val = 25000000
        prescale_val //= 4096
        prescale_val //= freq
        prescale_val -= 1

        prescale_val = floor(prescale_val + 0.5)

        current_mode = ord(
            self.i2c.mem_read(1, self.address, self.register['MODE1']))
        sleep_mode = (current_mode & 0x7f) | 0x10
        self.i2c.mem_write(sleep_mode, self.address, self.register['MODE1'])
        self.i2c.mem_write(prescale_val, self.address,
                           self.register['PRESCALE'])
        self.i2c.mem_write(current_mode, self.address, self.register['MODE1'])
        pyb.delay(5)
        self.i2c.mem_write(current_mode | 0xa1, self.address,
                           self.register['MODE1'])

    def reset(self):
        self.i2c.mem_write(0, self.address, self.register['MODE1'])

    def set_servo(self, servo_num, position):
        if servo_num <= 15:
            value = self.pos_to_pwm(position)
            self.i2c.mem_write(bytearray([0, 0, value & 0xff, value >> 8]),
                               self.address,
                               self.register['LED0'] + 4 * servo_num)

    def pos_to_pwm(self, position):
        # m * (x - x0) + y0 = y
        return int((self.servo_max - self.servo_min) / 180 * (
        position + 90) + self.servo_min)
