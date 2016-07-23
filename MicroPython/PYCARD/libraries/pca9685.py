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
SERVO_MAX_VALUE = 4095
SERVO_POSSIBLE_VALUES = 4096
SERVO_MIN_PULSE = 150
SERVO_MAX_PULSE = 600
SERVO_MIN = -90
SERVO_MAX = 90


class PCA9685:
    def __init__(self, bus, pwm_freq, address=0x40):
        self.address = address
        self.i2c = pyb.I2C(bus, pyb.I2C.MASTER)
        self.reset()

        self._set_pwm_freq(pwm_freq)

    def set_servo(self, servo_num, servo_value):
        self._set_pwm(servo_num, 0, self._map_value(servo_value))

    def _map_value(self, value):
        return int((SERVO_MAX_PULSE - SERVO_MIN_PULSE) /
                   (SERVO_MAX - SERVO_MIN) *
                   (value - SERVO_MIN) + SERVO_MIN_PULSE)

    def _set_pwm_freq(self, freq):
        freq *= 0.9
        pre_scale_val = 25000000
        pre_scale_val /= 4096
        pre_scale_val /= freq
        pre_scale_val -= 1

        pre_scale = math.floor(pre_scale_val + 0.5)

        old_mode = self.read_8(PCA9685_MODE1)
        new_mode = (old_mode & 0x7F) | 0x10

        self.write_8(PCA9685_MODE1, new_mode)
        self.write_8(PCA9685_PRESCALE, pre_scale)
        self.write_8(PCA9685_MODE1, old_mode)
        pyb.delay(5)
        self.write_8(PCA9685_MODE1, old_mode | 0xa1)

    def _set_pwm(self, num, on, off):
        data = [
            LED0_ON_L + 4 * num,
            on & 0xff,
            on >> 8,
            off & 0xff,
            off >> 8
        ]
        self.i2c.send(data, self.address)

    def _set_pin(self, num, value, invert):
        value = min(value, SERVO_MAX_VALUE)
        if invert:
            if value == 0:  # Special value for signal fully on.
                self._set_pwm(num, SERVO_POSSIBLE_VALUES, 0)
            elif value == SERVO_MAX_VALUE:  # Special value for signal fully off.
                self._set_pwm(num, 0, SERVO_POSSIBLE_VALUES)
            else:
                self._set_pwm(num, 0, SERVO_MAX_VALUE - value)
        else:
            if value == SERVO_MAX_VALUE:  # Special value for signal fully on.
                self._set_pwm(num, SERVO_POSSIBLE_VALUES, 0)
            elif value == 0:  # Special value for signal fully off.
                self._set_pwm(num, 0, SERVO_POSSIBLE_VALUES)
            else:
                self._set_pwm(num, 0, value)

    def reset(self):
        self.write_8(PCA9685_MODE1, 0x0)

    def write_8(self, register, data):
        return self.i2c.mem_write(data, self.address, register)

    def read_8(self, register):
        return self.i2c.mem_read(1, self.address, register)

    def read_len(self, register, length):
        return self.i2c.mem_read(length, self.address, register)

    def __len__(self):
        return 16  # num of servos on driver
