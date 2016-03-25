import pyb
from pyb import I2C

from libraries.micro_gps import MicropyGPS

# from libraries.mpu6050 import MPU6050
# from libraries.hmc5883l import HMC5883L
from libraries.bno055 import BNO055
# from libraries.pca9685 import PCA9685

from data import *


class GPS(Sensor):
    def __init__(self, sensor_id):
        super().__init__(sensor_id, ['f', 'f'],
            ["lat deg", "lat min", "long deg", "long min",
             "speed x", "speed y", "speed z",
             "heading", "altitude",
             "satellites in view", "satellites in use",
             "hdop", "pdop", "vdop"])

        self.gps_ref = MicropyGPS()

    def update(self, character):
        self.gps_ref.update(character)

    def update_data(self):
        return (
            self.gps_ref.latitude[0] + self.gps_ref.latitude[1] / 60,
            self.gps_ref.longitude[0] + self.gps_ref.longitude[1] / 60
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
    def __init__(self, sensor_id, bus, declination=(9, 17)):
        super().__init__(sensor_id,
                         ['f', 'f', 'f', 'f', 'f'],
                         ["accel x", "accel y", "accel z",
                          "gyro x", "gyro y", "gyro z",
                          "euler x", "euler y", "euler z",
                          "quat w", "quat x", "quat y", "quat z",
                          "temperature",
                          "grav x", "grav y", "grav z",
                          "mag x", "mag y", "mag z",
                          "compass"])
        self.bno = BNO055(bus)
        self.mass_data = self.update_all()

    def update_all(self):
        self.mass_data = (
            self.bno.get_lin_accel() +  # 0: x, 1: y, 2: z
            self.bno.get_gyro() +  # 3: x, 4: y, 5: z
            self.bno.get_euler() +  # 6: x, 7: y, 8: z
            self.bno.get_quat() +  # 9: w, 10: x, 11: y, 12: z
            (self.bno.get_temp(),) +  # 13
            self.bno.get_grav() +   # 14: x, 15: y, 16: z
            self.bno.get_mag() +   # 17: x, 18: y, 19: z
            (self.bno.get_heading(),)  # 20
        )

    def update_data(self):
        self.update_all()
        return self.mass_data[0], self.mass_data[1], self.mass_data[5], self.mass_data[8], self.mass_data[20]

    def update_log(self):
        return self.mass_data


class HallEncoder(Sensor):
    def __init__(self, sensor_id, analog_pin):
        super(HallEncoder, self).__init__(sensor_id, 'u64', 'counts')

        self.pin_ref = (
            pyb.ADC(pyb.Pin(analog_pin, pyb.Pin.ANALOG)))

        self.in_range = False
        self.enc_dist = 0
        self.timer1 = pyb.Timer(4, freq=50)
        self.timer1.callback(lambda t: self.on_interrupt())

        # need to be calibrated to real life values
        self.upper_threshold = 3600
        self.lower_threshold = 3300

    def on_interrupt(self):
        self.hall_value = self.pin_ref.read()

        if (self.in_range and (self.hall_value > self.upper_threshold)):
            self.in_range = False
            self.enc_dist += 1
        elif (not self.in_range and (self.hall_value <= self.lower_threshold)):
            self.in_range = True

    def update_data(self):
        return self.enc_dist

    def update_log(self):
        return self.enc_dist

class Servo(Command):
    def __init__(self, command_id, pin_num, start_pos=None):
        super().__init__(command_id, 'i8', 'angle')
        self.servo_ref = pyb.Servo(pin_num)
        if start_pos is not None:
            self.servo_ref.angle(start_pos)
        self.angle = start_pos

    def callback(self, angle):
        self.angle = angle
        self.servo_ref.angle(self.angle)

    def update_log(self):
        return self.angle

# class MCP9808(Sensor):
#     CONFIG = 0x01
#     CONFIG_SHUTDOWN = 0x0100
#     CONFIG_CRITLOCKED = 0x0080
#     CONFIG_WINLOCKED = 0x0040
#     CONFIG_INTCLR = 0x0020
#     CONFIG_ALERTSTAT = 0x0010
#     CONFIG_ALERTCTRL = 0x0008
#     CONFIG_ALERTSEL = 0x0004
#     CONFIG_ALERTPOL = 0x0002
#     CONFIG_ALERTMODE = 0x0001
#     UPPER_TEMP = 0x02
#     LOWER_TEMP = 0x03
#     CRIT_TEMP = 0x04
#     AMBIENT_TEMP = 0x05
#     MANUF_ID = 0x06
#     DEVICE_ID = 0x07
#
#     def __init__(self, sensor_id, bus, addr=0x18):
#         super().__init__(sensor_id, 'f')
#         self.bus = bus
#         self.addr = addr
#
#         self.i2c_ref = I2C(self.bus, I2C.MASTER)
#
#         addresses = self.i2c_ref.scan()
#         # print("Scanning devices:", [hex(x) for x in addresses])
#         if self.addr not in addresses:
#             raise Exception("MCP9808 is not detected")
#
#         manuf_id = self.i2c_read_16(self.MANUF_ID)
#         if manuf_id != 0x0054:
#             raise Exception("Invalid manufacture ID!", manuf_id)
#
#         device_id = self.i2c_read_16(self.DEVICE_ID)
#         if device_id != 0x0400:
#             raise Exception("Invalid device ID!", device_id)
#
#             # print("MCP9808 initialized!")
#
#     def i2c_read_16(self, register):
#         raw_bytes = self.i2c_ref.mem_read(2, self.addr, register)
#         return struct.unpack(">h", raw_bytes)[0]
#
#     def get_raw(self):
#         return self.i2c_read_16(self.AMBIENT_TEMP)
#
#     def read(self):
#         raw = self.get_raw()
#
#         temperature = raw & 0x0fff
#         temperature /= 16.0
#         if raw & 0x1000:
#             temperature -= 0x100
#
#         return temperature
#
#     def update_data(self):
#         return self.read()
#
#
# class TMP36(Sensor):
#     def __init__(self, sensor_id, adc_pin, vcc=3.3):
#         super().__init__(sensor_id, 'f')
#
#         self.pin_ref = pyb.ADC(pyb.Pin(adc_pin, pyb.Pin.ANALOG))
#         self.vcc = vcc * 1000
#
#     def read(self):
#         raw = self.pin_ref.read()
#         millivolts = self.vcc / 1024 * raw
#         return (1.0 / 37 * millivolts) - 55  # (millivolts - 500) / 100
#
#     def update_data(self):
#         return self.pin_ref.read()
#
#
# class BuiltInAccel(Sensor):
#     def __init__(self, sensor_id):
#         super().__init__(sensor_id, 'i8', 'i8', 'i8')
#
#         self.accel = pyb.Accel()
#
#     def update_data(self):
#         return (self.accel.x(), self.accel.y(), self.accel.z())


# servo_controller = None
#
#
# class I2CServo(Command):
#     def __init__(self, bus, command_id, servo_num):
#         global servo_controller
#         super().__init__(command_id, 'i8')
#         if servo_controller is None:
#             servo_controller = PCA9685(bus)
#         self.servo_num = servo_num
#
#     def callback(self, angle):
#         global servo_controller
#         pyb.LED(self.servo_num + 1).toggle()
#         servo_controller.set_servo(self.servo_num, angle)


# class RotaryEncoder(Sensor):
#     def __init__(self, sensor_id, pin_x, pin_y, pin_mode=pyb.Pin.PULL_NONE,
#                  scale=1, min=None, max=None, reverse=False):
#         super().__init__(sensor_id, 'i64')
#
#         self.encoder = MicroEncoder(pin_x, pin_y, pin_mode, scale, min, max,
#                                     reverse)
#
#     def update_data(self):
#         return self.encoder.position

# class AccelGyro(Sensor):
#     def __init__(self, sensor_id, bus):
#         super().__init__(sensor_id,
#                          'f', 'f', 'f',
#                          'f', 'f', 'f')
#         self.imu = MPU6050(bus, False)
#
#         self.accel_scale = 9.81 / 7900
#         self.gyro_scale = 1 / 7150
#
#     def scale(self, data, scale):
#         for index in range(len(data)):
#             data[index] *= scale
#
#     def interpret_buf(self, buf):
#         x = buf[0] << 8 | buf[1]
#         y = buf[2] << 8 | buf[3]
#         z = buf[4] << 8 | buf[5]
#         if x >> 15 == 1:
#             x -= 2 << 15
#         if y >> 15 == 1:
#             y -= 2 << 15
#         if z >> 15 == 1:
#             z -= 2 << 15
#
#         return [x, y, z]
#
#     def update_data(self):
#         accel = self.interpret_buf(self.imu.get_accel_raw())
#         gyro = self.interpret_buf(self.imu.get_gyro_raw())
#
#         self.scale(accel, self.accel_scale)
#         self.scale(gyro, self.gyro_scale)
#
#         return accel + gyro


# class Compass(Sensor):
#     def __init__(self, sensor_id, bus):
#         super().__init__(sensor_id, 'f')
#         self.compass = HMC5883L(bus, declination=(-9, 16))
#
#     def update_data(self):
#         return self.compass.heading()


# class Motor(Command):
#     # pin name: [(timer #, channel #), ...]
#     pin_channels = {
#         'X1': [(2, 1), (5, 1)],
#         'X2': [(2, 2), (5, 2)],
#         'X3': [(2, 3), (5, 3), (9, 1)],
#         'X4': [(2, 4), (5, 4), (9, 2)],
#         'X6': [(2, 1), (8, 1)],
#         'X7': [(13, 1)],
#         'X8': [(1, 1), (8, 1), (14, 1)],
#         'X9': [(4, 1)],
#         'X10': [(4, 2)],
#         'Y1': [(8, 1)],
#         'Y2': [(8, 2)],
#         'Y3': [(4, 3), (10, 1)],
#         'Y4': [(4, 4), (11, 1)],
#         'Y6': [(1, 1)],
#         'Y7': [(1, 2), (8, 2), (12, 1)],
#         'Y8': [(1, 3), (8, 3), (12, 2)],
#         'Y9': [(2, 3)],
#         'Y10': [(2, 4)],
#         'Y11': [(1, 2), (8, 2)],
#         'Y12': [(1, 3), (8, 3)]
#     }
#
#     def __init__(self, command_id, direction_pin, pwm_pin, min_speed=40,
#                  max_speed=100):
#         self.enable_pin = pyb.Pin(direction_pin, mode=pyb.Pin.OUT_PP)
#         self.timer, self.channel = self.init_timer_channel(pwm_pin, 100)
#
#         self.current_speed = 0
#         self.min_speed = min_speed
#         self.max_speed = max_speed
#
#         super().__init__(command_id, 'i8')
#
#     def init_timer_channel(self, pwm_pin, frequency):
#         if pwm_pin in self.pin_channels:
#             timer_num, channel_num = self.pin_channels[pwm_pin][0]
#
#             timer = pyb.Timer(timer_num, freq=frequency)
#             channel = timer.channel(channel_num, pyb.Timer.PWM,
#                                     pin=pyb.Pin(pwm_pin))
#
#             return timer, channel
#         else:
#             raise Exception("Not valid pin: " + str(pwm_pin))
#
#     def constrain_speed_abs(self, value):
#         if value == 0:
#             return value
#
#         if abs(value) < self.min_speed:
#             return self.min_speed
#
#         if abs(value) > self.max_speed:
#             return self.max_speed
#
#         return abs(value)
#
#     def speed(self, value=None):
#         if value == None:
#             return self.current_speed
#         else:
#             if value == 0:
#                 self.enable_pin.value(0)
#                 self.channel.pulse_width_percent(0)
#             elif value < 0:
#                 self.enable_pin.value(1)
#                 self.channel.pulse_width_percent(
#                     100 - self.constrain_speed_abs(value))
#             else:
#                 self.enable_pin.value(0)
#                 self.channel.pulse_width_percent(
#                     self.constrain_speed_abs(value))
#             self.current_speed = value
#
#     def callback(self, value):
#         self.speed(value)