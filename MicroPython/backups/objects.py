import math

import pyb

from data import *
from libraries.bno055 import BNO055
from libraries.micro_gps import MicropyGPS


class GPS(Sensor):
    def __init__(self, sensor_id):
        super().__init__(sensor_id, ['f', 'f', 'b'],
                         ["lat deg", "lat min", "long deg", "long min",
                          "speed x", "speed y", "speed z",
                          "heading", "altitude",
                          "satellites in view", "satellites in use",
                          "hdop", "pdop", "vdop"])

        self.gps_ref = MicropyGPS()

        self.prev_lat = None
        self.prev_long = None
        self.lat = 0.0
        self.long = 0.0

    def update(self, character):
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

    def update_log(self):
        return (
            self.gps_ref.latitude[0:2] +
            self.gps_ref.longitude[0:2] +
            self.gps_ref.speed +
            (self.gps_ref.course,
             self.gps_ref.altitude,
             self.gps_ref.satellites_in_view,
             self.gps_ref.satellites_in_use,
             self.gps_ref.hdop,
             self.gps_ref.pdop,
             self.gps_ref.vdop)
        )


class IMU(Sensor):
    def __init__(self, sensor_id, bus):
        super().__init__(sensor_id,
                         'f',
                         ["accel x", "accel y", "accel z",
                          "gyro x", "gyro y", "gyro z",
                          "euler x", "euler y", "euler z",
                          "quat w", "quat x", "quat y", "quat z",
                          "temperature",
                          "grav x", "grav y", "grav z",
                          "mag x", "mag y", "mag z",
                          "compass"])
        self.bus = bus
        self.bno = BNO055(self.bus)
        self.mass_data = tuple()
        self.update_all()

    def update_all(self):
        # self.mass_data = (
        #     self.bno.get_lin_accel() +  # 0: x, 1: y, 2: z
        #     self.bno.get_gyro() +  # 3: x, 4: y, 5: z
        #     self.bno.get_euler() +  # 6: z, 7: y, 8: x
        #     self.bno.get_quat() +  # 9: w, 10: x, 11: y, 12: z
        #     (self.bno.get_temp(),) +  # 13
        #     self.bno.get_grav() +   # 14: x, 15: y, 16: z
        #     self.bno.get_mag() +   # 17: x, 18: y, 19: z
        #     (self.bno.get_heading(),)  # 20
        # )
        self.mass_data = \
            self.bno.get_lin_accel()[0:2] + (
                self.bno.get_euler()[0] * math.pi / 180,
                self.bno.get_heading(),
            )

    def update_data(self):
        # self.update_all()
        # return (self.mass_data[0], self.mass_data[1], self.mass_data[5],
        #         self.mass_data[8], self.mass_data[20])
        # return self.mass_data
        return self.bno.get_euler()[0] * math.pi / 180

    def update_log(self):
        return self.mass_data


class HallEncoder(Sensor):
    def __init__(self, sensor_id, analog_pin):
        super(HallEncoder, self).__init__(sensor_id, 'u64', 'counts')

        self.pin_ref = pyb.ADC(pyb.Pin(analog_pin, pyb.Pin.ANALOG))

        self.in_range = False
        self.enc_dist = 0
        self.timer1 = pyb.Timer(4, freq=50)
        self.timer1.callback(lambda t: self.on_interrupt())

        # need to be calibrated to real life values
        self.upper_threshold = 3100
        self.lower_threshold = 2900

        self.data_recved = False

    def on_interrupt(self):
        self.hall_value = self.pin_ref.read()

        if self.in_range and (self.hall_value > self.upper_threshold):
            self.in_range = False
            self.enc_dist += 1
            self.data_recved = True
        elif not self.in_range and (self.hall_value <= self.lower_threshold):
            self.in_range = True

    def update_data(self):
        return self.enc_dist

    def update_log(self):
        return self.enc_dist

    def reset(self):
        self.enc_dist = 0

    def recved_data(self):
        if self.data_recved == True:
            self.data_recved = False
            return True
        else:
            return False


class Servo(Command):
    def __init__(self, command_id, pin_num, start_pos=0):
        super().__init__(command_id, 'i8', 'angle')
        self.start_pos = start_pos
        self.servo_ref = pyb.Servo(pin_num)
        if start_pos is not None:
            self.servo_ref.angle(start_pos)
        self.angle = start_pos

    def callback(self, angle):
        self.angle = angle
        self.servo_ref.angle(self.angle)

    def update_log(self):
        return self.angle

    def reset(self):
        self.angle = self.start_pos
        self.servo_ref.angle(self.angle)
