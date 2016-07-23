from math import pi

import pyb

from data import *
from libraries.adafruit_gps import AdafruitGPS
from libraries.bno055 import BNO055
from libraries.pca9685 import PCA9685

used_timers = {}


def add_timer(timer_num, timer_freq):
    global used_timers
    if timer_num not in used_timers:
        used_timers[timer_num] = timer_freq
    elif used_timers[timer_num] != timer_freq:
        # if frequencies don't match
        raise ValueError("Timer already used:", timer_num, used_timers)


class GPS(Sensor):
    def __init__(self, sensor_id, uart_bus, timer_num, baud_rate=9600,
                 update_rate=5):
        super(GPS, self).__init__(sensor_id, ['f', 'f', 'b'])
        self.gps_ref = AdafruitGPS(uart_bus, timer_num, baud_rate, update_rate)

        add_timer(timer_num, self.gps_ref.timer.freq())

    def recved_data(self):
        return self.gps_ref.received_sentence()

    def update_data(self):
        return self.gps_ref.longitude, self.gps_ref.latitude, self.gps_ref.fix


class IMU(Sensor):
    def __init__(self, sensor_id, bus, timer_num):
        super(IMU, self).__init__(sensor_id, 'f')
        self.bus = bus
        self.bno = BNO055(self.bus)

        self.new_data = False
        self.prev_yaw = 0.0

        self.timer = pyb.Timer(timer_num, freq=100)
        self.timer.callback(lambda _: self.callback())

        add_timer(timer_num, self.timer.freq())

    def get_yaw(self):
        return -self.bno.get_euler()[0] * pi / 180

    def recved_data(self):
        if self.new_data:
            self.new_data = False
            self.yaw = self.get_yaw()
            if self.prev_yaw != self.yaw:
                self.prev_yaw = self.yaw
                return True
        return False

    def update_data(self):
        return self.yaw

    def callback(self):
        self.new_data = True


class ServoDriver(Command):
    def __init__(self, command_id, i2c_bus, pwm_freq, initial_state=0):
        super().__init__(command_id)
        self.initial_state = initial_state
        self.driver = PCA9685(i2c_bus, pwm_freq)

        self.set_all(self.initial_state)

    def set_all(self, value):
        for servo_num in range(len(self.driver)):
            self.driver.set_servo(servo_num, value)

    def callback(self, data):
        self.driver.set_servo(data[0], data[1])

    def reset(self):
        self.set_all(self.initial_state)


class PybLEDs(Command):
    def __init__(self, command_id, initial_state=0):
        super().__init__(command_id)
        self.leds = [pyb.LED(led_num) for led_num in range(1, 5)]
        self.initial_state = initial_state
        self.set_all(initial_state)

    def set_all(self, state):
        for led_num in range(1, 5):
            self.set_state(led_num, state)

    def set_state(self, led_num, state):
        led_num -= 1
        if led_num == 3:
            self.leds[led_num].intensity(state)
        else:
            if state == 0:
                self.leds[led_num].off()
            elif state == 1:
                self.leds[led_num].on()
            elif state == 2:
                self.leds[led_num].toggle()

    def callback(self, data):  # led num, value
        self.set_state(data[0], data[1])

    def reset(self):
        self.set_all(self.initial_state)


class MotorCommand(Command):
    def __init__(self, command_id, rc_motor):
        super().__init__(command_id)
        self.rc_motor = rc_motor

    def callback(self, speed):
        self.rc_motor.set_speed(speed)

    def reset(self):
        self.rc_motor.set_speed(0)


class RCencoder(Sensor):
    def __init__(self, sensor_id, rc_motor):
        super().__init__(sensor_id, 'i64')
        self.rc_motor = rc_motor

        for timer_num, timer in self.rc_motor.timers.items():
            add_timer(timer_num, timer.freq())

    def reset(self):
        self.rc_motor.enc_dist = 0

    def update_data(self):
        return self.rc_motor.enc_dist

    def recved_data(self):
        return self.rc_motor.new_encoder_data()
