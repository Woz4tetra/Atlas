from math import pi

import pyb
import time

from data import *
from libraries.adafruit_gps import AdafruitGPS
from libraries.bno055 import BNO055
from libraries.stepper import Stepper

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
        super(GPS, self).__init__(sensor_id, ['f'] * 7 + ['b'])
        self.gps_ref = AdafruitGPS(uart_bus, timer_num, baud_rate, update_rate)

        add_timer(timer_num, self.gps_ref.timer.freq())

    def stop(self):
        print("GPS entering standby... ", end="")
        self.gps_ref.standby()
        print("done!")

    def reset(self):
        print("GPS waking up... ", end="")
        self.gps_ref.wakeup()
        print("done!")


    def recved_data(self):
        return self.gps_ref.received_sentence()

    def update_data(self):
        return (self.gps_ref.latitude, self.gps_ref.longitude,
                self.gps_ref.altitude, self.gps_ref.geoid_height,
                self.gps_ref.pdop, self.gps_ref.hdop, self.gps_ref.vdop,
                self.gps_ref.fix)


class IMU(Sensor):
    def __init__(self, sensor_id, bus, reset_pin, timer_num):
        super(IMU, self).__init__(sensor_id,
                                  ['f', 'f', 'f', 'f', 'f', 'f', 'f', 'f', 'f', 'f'])
#                                  + ['u8'] * 11)
        self.bus = bus
        print("IMU bus:", self.bus)
        self.bno = BNO055(self.bus, reset_pin)
        print("IMU reset pin:", reset_pin)

        self.new_data = False

        self.yaw = 0.0
        self.accel_x, self.accel_y, self.accel_z = 0.0, 0.0, 0.0
        self.ang_vx, self.ang_vy, self.ang_vz = 0.0, 0.0, 0.0
        self.mag_x, self.mag_y, self.mag_z = 0.0, 0.0, 0.0

    def reset(self):
        print("Resetting IMU... ", end="")
        self.bno.reset()
        print("done!")

    def recved_data(self):
        self.yaw = self.bno.get_euler()[0] * pi / 180  # radians

        self.accel_x, self.accel_y, self.accel_z = self.bno.get_lin_accel()  # m/s^2

        self.ang_vx, self.ang_vy, self.ang_vz = self.bno.get_gyro()  # radians per second
        
#        ang_v = self.bno.get_gyro()  # radians per second
#        self.ang_vx = ang_v[0] * 2 * pi  # radians per second
#        self.ang_vy = ang_v[1] * 2 * pi
#        self.ang_vz = ang_v[2] * 2 * pi

        self.mag_x, self.mag_y, self.mag_z = self.bno.get_mag()

#        self.bno.update_offsets()

        return True

    def update_data(self):

        return (self.yaw, self.accel_x, self.accel_y, self.accel_z,
                self.ang_vx, self.ang_vy, self.ang_vz,
                self.mag_x, self.mag_y, self.mag_z
                ) # + tuple(self.bno.offsets.values())


class StepperCommand(Command):
    def __init__(self, command_id):
        super().__init__(command_id, 'i16')

        self.delimiter_pin = pyb.Pin("Y1", pyb.Pin.IN, pyb.Pin.PULL_UP)

        self.stepper = Stepper(200, 25, "Y3", "Y4", "Y5", "Y6")

    def callback(self, steps):
        self.stepper.step(steps)

    def calibrate(self):
        print("Calibrating stepper... ", end="")
        while not self.delimiter_pin.value():
            self.stepper.step(-5)
        
        pyb.delay(10)  # if this isn't here the stepper won't switch directions
        
        self.stepper.step(150)  # center the steering
        print("done!")

    def reset(self):
        # recalibrate with delimiter
        self.calibrate()


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
        self.led = pyb.LED(4)
        self.set_state(initial_state)

    def set_state(self, state):
        self.led.intensity(state)

    def callback(self, state):
        self.set_state(state)

    def reset(self):
        self.set_state(0)
