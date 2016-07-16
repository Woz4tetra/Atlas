from math import pi

import pyb

from data import *
from libraries.adafruit_gps import AdafruitGPS
from libraries.bno055 import BNO055

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
        super(GPS, self).__init__(sensor_id, ['f', 'f'])
        self.gps_ref = AdafruitGPS(uart_bus, timer_num, baud_rate, update_rate)

        add_timer(timer_num, self.gps_ref.timer.freq())

    def recved_data(self):
        return self.gps_ref.received_sentence()

    def update_data(self):
        return self.gps_ref.longitude, self.gps_ref.latitude


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


class ServoCommand(Command):
    def __init__(self, command_id, pin_num, start_pos=0):
        super().__init__(command_id, 'i8')
        self.start_pos = start_pos
        self.servo_ref = pyb.Servo(pin_num)
        if start_pos is not None:
            self.servo_ref.angle(start_pos)
        self.angle = start_pos

    def callback(self, angle):
        self.angle = angle
        self.servo_ref.angle(self.angle)

    def reset(self):
        self.angle = self.start_pos
        self.servo_ref.angle(self.angle)


class LEDcommand(Command):
    def __init__(self, command_id, led_num, initial_state=0):
        super().__init__(command_id, 'u4')
        self.led = pyb.LED(led_num)
        self.set_state(initial_state)

    def set_state(self, state):
        if state == 0:
            self.led.off()
        elif state == 1:
            self.led.on()
        elif state == 2:
            self.led.toggle()

    def callback(self, state):
        self.set_state(state)

    def reset(self):
        self.set_state(0)


class BlueLEDcommand(Command):
    def __init__(self, command_id, initial_state=0):
        super().__init__(command_id, 'u8')
        self.led = pyb.LED(led_num)
        self.set_state(initial_state)

    def set_state(self, state):
        led.intensity(state)

    def callback(self, state):
        self.set_state(state)

    def reset(self):
        self.set_state(0)


class MotorCommand(Command):
    def __init__(self, command_id, rc_motor):
        super().__init__(command_id, 'i8')
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
