import math

import pyb

from data import *
from libraries.bno055 import BNO055
from libraries.micro_gps import MicropyGPS
from libraries.rc_motors import RCmotors

class GPS(Sensor):
    gps_indicator = pyb.LED(3)
    new_data = False
    uart = None
    pps_pin = None
    extint = None
    
    def __init__(self, sensor_id, uart_bus, int_pin):
        super().__init__(sensor_id, ['f', 'f', 'b'])

        self.gps_ref = MicropyGPS()

        self.prev_lat = None
        self.prev_long = None
        self.lat = 0.0
        self.long = 0.0
        
        # using class variables because lists can't be created in callbacks
        # MicropyGPS creates a list in one of its functions
        GPS.pps_pin = int_pin
        GPS.ext_pin = pyb.ExtInt(GPS.pps_pin, pyb.ExtInt.IRQ_FALLING,
                        pyb.Pin.PULL_UP, GPS.pps_callback)
        GPS.uart = pyb.UART(6, 9600, read_buf_len=1000)


    def update_gps(self, character):
        self.gps_ref.update(character)

    def heading(self):
        if self.prev_lat is None and self.prev_long is None:
            self.prev_lat = self.lat
            self.prev_long = self.long
            return 0.0
        else:
            angle = math.atan2(self.long - self.prev_long, self.lat - self.prev_lat)
            self.prev_long = self.long
            self.prev_lat = self.lat
            return angle

    def update_data(self):
        self.lat = self.gps_ref.latitude[0] + self.gps_ref.latitude[1] / 60
        self.long = self.gps_ref.longitude[0] + self.gps_ref.longitude[1] / 60
        return (
            self.lat,
            self.long,
            self.gps_ref.satellites_in_view > 0
        )
    
    def recved_data(self):
        return GPS.new_data
    
    @staticmethod
    def pps_callback(line):
        GPS.new_data = True
        GPS.gps_indicator.toggle()

    def stream_data(self):
        while GPS.uart.any():
            self.update_gps(chr(GPS.uart.readchar()))
        GPS.new_data = False

class IMU(Sensor):
    def __init__(self, sensor_id, bus):
        super().__init__(sensor_id, 'f')
        self.bus = bus
        self.bno = BNO055(self.bus)

    def update_data(self):
        return self.bno.get_euler()[0] * math.pi / 180


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

    def reset(self):
        self.rc_motor.enc_dist = 0
    
    def update_data(self):
        return self.rc_motor.enc_dist
    
    def recved_data(self):
        return self.rc_motor.new_encoder_data()

