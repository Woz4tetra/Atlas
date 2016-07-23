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
    def __init__(self, bus, address=0x40):
        self.i2c = pyb.I2C(bus, pyb.I2C.MASTER)
        self.address = address
        self.reset()

    def set_servo(self, servo_num, ):

    def set_pwm_freq(self, freq):
        # Correct for overshoot in the frequency setting
        freq *= 0.9
        prescaleval = 25000000
        prescaleval /= 4096
        prescaleval /= freq
        prescaleval -= 1

        if ENABLE_DEBUG_OUTPUT:
            print("Estimated pre-scale:", prescaleval)

        prescale = math.floor(prescaleval + 0.5)

        if ENABLE_DEBUG_OUTPUT:
            print("Final pre-scale:", prescale)

        old_mode = self.read_8(PCA9685_MODE1)
        new_mode = (old_mode & 0x7F) | 0x10  # sleep
        self.write_8(PCA9685_MODE1, new_mode)  # go to sleep
        self.write_8(PCA9685_PRESCALE, prescale)  # set the prescaler
        self.write_8(PCA9685_MODE1, old_mode)
        pyb.delay(5)

        # This sets the MODE1 register to turn on auto increment.
        self.write_8(PCA9685_MODE1, old_mode | 0xa1)

    def reset(self):
        self.write_8(PCA9685_MODE1, 0)

    def write_8(self, register, data):
        return self.i2c.mem_write(data, self.address, register)

    def read_8(self, register):
        return self.i2c.mem_read(1, self.address, register)

    def read_len(self, register, length):
        return self.i2c.mem_read(length, self.address, register)

    def __len__(self):
        return 16  # num of servos on driver
