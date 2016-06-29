import math
import time
import pyb

from data import *
from libraries.bno055 import BNO055
from libraries.micro_gps import MicropyGPS
from libraries.rc_motors import RCmotors
from libraries.bmp280 import *

class GPS(Sensor):
    gps_indicator = pyb.LED(3)
    new_data = False
    uart = None
    pps_pin = None
    extint = None
    
    def __init__(self, sensor_id, uart_bus, int_pin=None, timer_num=None):
        assert not (int_pin == timer_num == None)
        super().__init__(sensor_id, ['f', 'f', 'f', 'u8'])
        
        self.gps_ref = MicropyGPS()

        self.prev_lat = None
        self.prev_long = None
        self.lat = 0.0
        self.long = 0.0
        self.altitude = 0.0

        GPS.uart = pyb.UART(uart_bus, 9600, read_buf_len=1000)
        
        # using class variables because lists can't be created in callbacks
        # MicropyGPS creates a list in one of its functions
        if int_pin is not None:
            GPS.pps_pin = int_pin
            GPS.ext_pin = pyb.ExtInt(GPS.pps_pin, pyb.ExtInt.IRQ_FALLING,
                            pyb.Pin.PULL_UP, GPS.pps_callback)
        else:
            self.timer = pyb.Timer(timer_num, freq=1)
            self.timer.callback(lambda t: GPS.pps_callback(t))

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
            -self.long,
##            self.gps_ref.geoid_height,
            self.gps_ref.altitude,
            self.gps_ref.satellites_in_view
        )
    
    def recved_data(self):
        if GPS.new_data:
            while GPS.uart.any():
                self.update_gps(chr(GPS.uart.readchar()))
            GPS.new_data = False
            return True
        else:
            return False
    
    @staticmethod
    def pps_callback(line):
        if GPS.uart.any():
            GPS.new_data = True
            GPS.gps_indicator.toggle()
        

class IMU(Sensor):
    def __init__(self, sensor_id, timer_num, bus, get_ang_v=True):
        super().__init__(sensor_id, 'f')
        self.bus = bus
        self.bno = BNO055(self.bus)
        
        self.yaw = self.get_yaw()
        self.ang_v = 0.0
        self.prev_ang_v = None
        
        self.new_data = False
        self.prev_time = time.ticks_us()
        
        self.timer = pyb.Timer(timer_num, freq=100)
        self.get_ang_v = get_ang_v
        if self.get_ang_v:
            self.timer.callback(lambda _: self.callback_angular_v(_))
        else:
            self.timer.callback(lambda _: self.callback_yaw(_))
    
    def get_yaw(self):
        return self.bno.get_euler()[0] * math.pi / 180
    
    def callback_yaw(self, line):
        new_yaw = self.get_yaw()
        if self.yaw != new_yaw:
            self.new_data = True
            self.yaw = new_yaw
    
    def callback_angular_v(self, line):
        dt = time.ticks_diff(self.prev_time, time.ticks_us()) / 1E6
        new_yaw = self.get_yaw()
        
        self.ang_v = (new_yaw - self.yaw) / dt 
        self.yaw = new_yaw
        if self.prev_ang_v != self.ang_v:
            self.new_data = True
            self.prev_ang_v = self.ang_v

    def recved_data(self):
        if self.new_data:
            self.new_data = False
            return True
        else:
            return False
    
    def update_data(self):
        if self.get_ang_v:
            return self.yaw
        else:
            return self.ang_v


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

class Altitude(Sensor):
    def __init__(self, sensor_id, bus, frequency=20, use_i2c=True):
        if use_i2c:
            self.bmp280 = BMP280_I2C(bus)
        else:
            self.bmp280 = BMP280_SPI(bus)
        self.counter = 0
        self.freq = frequency
        super(Altitude, self).__init__(sensor_id, 'f')

    def update_data(self):
        return self.bmp280.altitude()

    def recved_data(self):
        self.counter += 1
        if self.counter == self.freq:
            self.counter = 0
            return True
        else:
            return False

