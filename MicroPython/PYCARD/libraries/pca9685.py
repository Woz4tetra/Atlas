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

import pyb
import math

PCA9685_SUBADR1 = 0x2
PCA9685_SUBADR2 = 0x3
PCA9685_SUBADR3 = 0x4
PCA9685_MODE1 = 0x0
PCA9685_PRESCALE = 0xFE
LED0_ON_L = 0x6
LED0_ON_H = 0x7
LED0_OFF_L = 0x8
LED0_OFF_H = 0x9
ALLLED_ON_L = 0xFA
ALLLED_ON_H = 0xFB
ALLLED_OFF_L = 0xFC
ALLLED_OFF_H = 0xFD
ENABLE_DEBUG_OUTPUT = False


class ServoDriver:
    def __init__(self, bus, servo_angle_min, servo_angle_max, servo_pulse_min,
                 servo_pulse_max, address=0x40, pwm_freq=60):
        self.i2c = pyb.I2C(bus, pyb.I2C.MASTER)
        self.address = address

        self.servo_angle_min = servo_angle_min
        self.servo_angle_max = servo_angle_max

        self.servo_pulse_min = servo_pulse_min
        self.servo_pulse_max = servo_pulse_max

        assert 0 <= self.servo_pulse_min < 4096
        assert 0 <= self.servo_pulse_max < 4096

        self.conversion = (self.servo_pulse_max - self.servo_pulse_min) / (
            self.servo_angle_max - self.servo_angle_min)

        addresses = self.i2c.scan()
        if self.address not in addresses:
            raise Exception("Address %s not found during scan: %s" % (
                self.address, addresses))
        if not self.i2c.is_ready(self.address):
            raise Exception("Device not ready")

        self.reset()

        assert 0 <= pwm_freq <= 1600
        self.set_pwm_freq(pwm_freq)

    def angle_to_pulse(self, angle):
        return int(self.conversion * (
            angle - self.servo_angle_min) + self.servo_pulse_min)

    def set_servo(self, servo_num, value):
        self.set_pwm(servo_num, 0, self.angle_to_pulse(value))

    def set_pwm(self, servo_num, on, off):
        self.write_numbers(LED0_ON_L + 4 * servo_num, on, off)

    def set_pwm_freq(self, freq):
        # Correct for overshoot in the frequency setting
        freq *= 0.9
        prescaleval = 25000000
        prescaleval /= 4096
        prescaleval /= freq
        prescaleval -= 1

        if ENABLE_DEBUG_OUTPUT:
            print("Estimated pre-scale:", prescaleval)

        prescale = int(math.floor(prescaleval + 0.5))

        if ENABLE_DEBUG_OUTPUT:
            print("Final pre-scale:", prescale)

        old_mode = self.read_8(PCA9685_MODE1)
        new_mode = (old_mode & 0x7F) | 0x10  # sleep

        assert new_mode < 0x100 and old_mode < 0x100 and prescale < 0x100

        self.write_8(PCA9685_MODE1, new_mode)  # go to sleep
        self.write_8(PCA9685_PRESCALE, prescale)  # set the prescaler
        self.write_8(PCA9685_MODE1, old_mode)
        pyb.delay(5)

        assert old_mode | 0xa1 < 0x100

        # This sets the MODE1 register to turn on auto increment.
        self.write_8(PCA9685_MODE1, old_mode | 0xa1)

    def reset(self):
        self.write_8(PCA9685_MODE1, 0)

    def write_numbers(self, *numbers):  # numbers are sent in list order
        transmission = []
        for number in numbers:
            while number > 0:
                # transmission += bytes(chr(number & 0xff), encoding='ascii')
                transmission.append(number & 0xff)
                number >>= 8
                # transmission += bytes(chr(number), encoding='utf-8')
        if ENABLE_DEBUG_OUTPUT:
            print("transmitting:", transmission)
        self.i2c.send(transmission, self.address)

    def write_8(self, register, data):
        if ENABLE_DEBUG_OUTPUT:
            print("transmitting 8-bit:", data & 0xff)
        return self.i2c.mem_write(data & 0xff, self.address, register)

    def read_8(self, register):
        return self.i2c.mem_read(1, self.address, register)

    def read(self, register, length):
        return self.i2c.mem_read(length, self.address, register)
