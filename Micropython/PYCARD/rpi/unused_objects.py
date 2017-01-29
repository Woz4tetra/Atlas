import pyb
from libraries.hmc5883l import HMC5883L
from libraries.mpu6050 import MPU6050

from data import *
from libraries.bmp280 import *
from libraries.micro_gps import MicropyGPS
from libraries.pca9685 import PCA9685


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


class GPS(Sensor):
    gps_indicator = pyb.LED(3)
    new_data = False
    uart = None
    pps_pin = None
    pps_timer = 0
    extint = None

    def __init__(self, sensor_id, uart_bus, int_pin, timer_num):
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
        GPS.pps_pin = int_pin
        GPS.ext_pin = pyb.ExtInt(GPS.pps_pin, pyb.ExtInt.IRQ_FALLING,
                                 pyb.Pin.PULL_UP, GPS.pps_callback)
        self.timer = pyb.Timer(timer_num, freq=1)
        self.timer.callback(lambda t: GPS.timer_callback(t))

    def update_gps(self, character):
        self.gps_ref.update(character)

        def heading(self):
            if self.prev_lat is None and self.prev_long is None:
                self.prev_lat = self.lat
                self.prev_long = self.long
                return 0.0
            else:
                angle = math.atan2(self.long - self.prev_long,
                                   self.lat - self.prev_lat)
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
        GPS.pps_timer += 1
        if GPS.new_data and GPS.pps_timer < 500:
            while GPS.uart.any():
                self.update_gps(chr(GPS.uart.readchar()))
            GPS.new_data = False
            return True
        else:
            return False

    @staticmethod
    def timer_callback(line):
        if GPS.uart.any():
            GPS.new_data = True
            GPS.gps_indicator.toggle()

    @staticmethod
    def pps_callback(line):
        GPS.pps_timer = 0


class HallEncoder(Sensor):
    def __init__(self, sensor_id, analog_pin, lower, upper):
        super(HallEncoder, self).__init__(sensor_id, 'u64')

        self.pin_ref = pyb.ADC(pyb.Pin(analog_pin, pyb.Pin.ANALOG))

        self.in_range = False
        self.enc_dist = 0
        self.hall_value = 0

        self.sum = 0
        self.count = 0

        # need to be calibrated to real life values
        # for RoboQuasar, upper = 3100, lower = 2900
        self.upper_threshold = upper
        self.lower_threshold = lower

        self.data_recved = False

        self.timer1 = pyb.Timer(1, freq=5000)
        self.timer1.callback(lambda t: self.on_interrupt())

        assert analog_pin == "X8" or analog_pin == "Y11" or analog_pin == "Y12"

    def on_interrupt(self):
        self.hall_value = self.pin_ref.read()
        self.sum += self.hall_value
        self.count += 1

        if self.count == 25:
            average = self.sum // self.count

            if self.in_range and (average > self.upper_threshold):
                self.in_range = False
                self.enc_dist += 1
                self.data_recved = True
            elif not self.in_range and (average <= self.lower_threshold):
                self.in_range = True

            self.sum = 0
            self.count = 0

    def update_data(self):
        return self.enc_dist

    def reset(self):
        self.enc_dist = 0

    def recved_data(self):
        if self.data_recved == True:
            self.data_recved = False
            return True
        else:
            return False


class MCP9808(Sensor):
    CONFIG = 0x01
    CONFIG_SHUTDOWN = 0x0100
    CONFIG_CRITLOCKED = 0x0080
    CONFIG_WINLOCKED = 0x0040
    CONFIG_INTCLR = 0x0020
    CONFIG_ALERTSTAT = 0x0010
    CONFIG_ALERTCTRL = 0x0008
    CONFIG_ALERTSEL = 0x0004
    CONFIG_ALERTPOL = 0x0002
    CONFIG_ALERTMODE = 0x0001
    UPPER_TEMP = 0x02
    LOWER_TEMP = 0x03
    CRIT_TEMP = 0x04
    AMBIENT_TEMP = 0x05
    MANUF_ID = 0x06
    DEVICE_ID = 0x07

    def __init__(self, sensor_id, bus, addr=0x18):
        super().__init__(sensor_id, 'f')
        self.bus = bus
        self.addr = addr

        self.i2c_ref = I2C(self.bus, I2C.MASTER)

        addresses = self.i2c_ref.scan()
        # print("Scanning devices:", [hex(x) for x in addresses])
        if self.addr not in addresses:
            raise Exception("MCP9808 is not detected")

        manuf_id = self.i2c_read_16(self.MANUF_ID)
        if manuf_id != 0x0054:
            raise Exception("Invalid manufacture ID!", manuf_id)

        device_id = self.i2c_read_16(self.DEVICE_ID)
        if device_id != 0x0400:
            raise Exception("Invalid device ID!", device_id)

            # print("MCP9808 initialized!")

    def i2c_read_16(self, register):
        raw_bytes = self.i2c_ref.mem_read(2, self.addr, register)
        return struct.unpack(">h", raw_bytes)[0]

    def get_raw(self):
        return self.i2c_read_16(self.AMBIENT_TEMP)

    def read(self):
        raw = self.get_raw()

        temperature = raw & 0x0fff
        temperature /= 16.0
        if raw & 0x1000:
            temperature -= 0x100

        return temperature

    def update_data(self):
        return self.read()


class TMP36(Sensor):
    def __init__(self, sensor_id, adc_pin, vcc=3.3):
        super().__init__(sensor_id, 'f')

        self.pin_ref = pyb.ADC(pyb.Pin(adc_pin, pyb.Pin.ANALOG))
        self.vcc = vcc * 1000

    def read(self):
        raw = self.pin_ref.read()
        millivolts = self.vcc / 1024 * raw
        return (1.0 / 37 * millivolts) - 55  # (millivolts - 500) / 100

    def update_data(self):
        return self.pin_ref.read()


class BuiltInAccel(Sensor):
    def __init__(self, sensor_id):
        super().__init__(sensor_id, 'i8', 'i8', 'i8')

        self.accel = pyb.Accel()

    def update_data(self):
        return (self.accel.x(), self.accel.y(), self.accel.z())


servo_controller = None


class I2CServo(Command):
    def __init__(self, bus, command_id, servo_num):
        global servo_controller
        super().__init__(command_id, 'i8')
        if servo_controller is None:
            servo_controller = PCA9685(bus)
        self.servo_num = servo_num

    def callback(self, angle):
        global servo_controller
        pyb.LED(self.servo_num + 1).toggle()
        servo_controller.set_servo(self.servo_num, angle)


class RotaryEncoder(Sensor):
    def __init__(self, sensor_id, pin_x, pin_y, pin_mode=pyb.Pin.PULL_NONE,
                 scale=1, min=None, max=None, reverse=False):
        super().__init__(sensor_id, 'i64')

        self.encoder = MicroEncoder(pin_x, pin_y, pin_mode, scale, min, max,
                                    reverse)

    def update_data(self):
        return self.encoder.position


class AccelGyro(Sensor):
    def __init__(self, sensor_id, bus):
        super().__init__(sensor_id,
                         'f', 'f', 'f',
                         'f', 'f', 'f')
        self.imu = MPU6050(bus, False)

        self.accel_scale = 9.81 / 7900
        self.gyro_scale = 1 / 7150

    def scale(self, data, scale):
        for index in range(len(data)):
            data[index] *= scale

    def interpret_buf(self, buf):
        x = buf[0] << 8 | buf[1]
        y = buf[2] << 8 | buf[3]
        z = buf[4] << 8 | buf[5]
        if x >> 15 == 1:
            x -= 2 << 15
        if y >> 15 == 1:
            y -= 2 << 15
        if z >> 15 == 1:
            z -= 2 << 15

        return [x, y, z]

    def update_data(self):
        accel = self.interpret_buf(self.imu.get_accel_raw())
        gyro = self.interpret_buf(self.imu.get_gyro_raw())

        self.scale(accel, self.accel_scale)
        self.scale(gyro, self.gyro_scale)

        return accel + gyro


class Compass(Sensor):
    def __init__(self, sensor_id, bus):
        super().__init__(sensor_id, 'f')
        self.compass = HMC5883L(bus, declination=(-9, 16))

    def update_data(self):
        return self.compass.heading()


class Motor(Command):
    # pin name: [(timer #, channel #), ...]
    pin_channels = {
        'X1': [(2, 1), (5, 1)],
        'X2': [(2, 2), (5, 2)],
        'X3': [(2, 3), (5, 3), (9, 1)],
        'X4': [(2, 4), (5, 4), (9, 2)],
        'X6': [(2, 1), (8, 1)],
        'X7': [(13, 1)],
        'X8': [(1, 1), (8, 1), (14, 1)],
        'X9': [(4, 1)],
        'X10': [(4, 2)],
        'Y1': [(8, 1)],
        'Y2': [(8, 2)],
        'Y3': [(4, 3), (10, 1)],
        'Y4': [(4, 4), (11, 1)],
        'Y6': [(1, 1)],
        'Y7': [(1, 2), (8, 2), (12, 1)],
        'Y8': [(1, 3), (8, 3), (12, 2)],
        'Y9': [(2, 3)],
        'Y10': [(2, 4)],
        'Y11': [(1, 2), (8, 2)],
        'Y12': [(1, 3), (8, 3)]
    }

    def __init__(self, command_id, direction_pin, pwm_pin, min_speed=40,
                 max_speed=100):
        self.enable_pin = pyb.Pin(direction_pin, mode=pyb.Pin.OUT_PP)
        self.timer, self.channel = self.init_timer_channel(pwm_pin, 100)

        self.current_speed = 0
        self.min_speed = min_speed
        self.max_speed = max_speed

        super().__init__(command_id, 'i8')

    def init_timer_channel(self, pwm_pin, frequency):
        if pwm_pin in self.pin_channels:
            timer_num, channel_num = self.pin_channels[pwm_pin][0]

            timer = pyb.Timer(timer_num, freq=frequency)
            channel = timer.channel(channel_num, pyb.Timer.PWM,
                                    pin=pyb.Pin(pwm_pin))

            return timer, channel
        else:
            raise Exception("Not valid pin: " + str(pwm_pin))

    def constrain_speed_abs(self, value):
        if value == 0:
            return value

        if abs(value) < self.min_speed:
            return self.min_speed

        if abs(value) > self.max_speed:
            return self.max_speed

        return abs(value)

    def speed(self, value=None):
        if value == None:
            return self.current_speed
        else:
            if value == 0:
                self.enable_pin.value(0)
                self.channel.pulse_width_percent(0)
            elif value < 0:
                self.enable_pin.value(1)
                self.channel.pulse_width_percent(
                    100 - self.constrain_speed_abs(value))
            else:
                self.enable_pin.value(0)
                self.channel.pulse_width_percent(
                    self.constrain_speed_abs(value))
            self.current_speed = value

    def callback(self, value):
        self.speed(value)


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
