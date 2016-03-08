# main.py -- put your code here!

import pyb
from pyb import UART
from objects import *
from data import *
from comm import Communicator

# tmp36 = TMP36(0, pyb.Pin.board.Y12)
# mcp9808 = MCP9808(1, 1)
# accel = BuiltInAccel(2)
gps = GPS(1)
# accel_gyro = AccelGyro(4, 1)
# compass = Compass(5, 1)
encoder = HallEncoder(2, "X7")
imu = IMU(3, 2)

# servo_steering = I2CServo(2, 0, 0)
# servo_brakes = I2CServo(2, 1, 1)

servo_steering = Servo(0, 1, 0)
# motor_a = Motor(1, 'X2', 'X3')

gps_indicator = pyb.LED(3)

new_data = False


def pps_callback(line):
    global new_data, gps_indicator
    new_data = True
    gps_indicator.toggle()


uart = UART(6, 9600, read_buf_len=1000)
pps_pin = pyb.Pin.board.X8
extint = pyb.ExtInt(pps_pin, pyb.ExtInt.IRQ_FALLING,
                    pyb.Pin.PULL_UP, pps_callback)

indicator = pyb.LED(4)
increase = True

sensor_queue = SensorQueue(
        gps,
	    encoder,
        imu
)
command_pool = CommandPool(
        servo_steering,
        # servo_brakes
)

communicator = Communicator(sensor_queue, command_pool)

while True:
    if new_data:
        while uart.any():
            gps.update(chr(uart.readchar()))

    new_data = False

    communicator.write_packet()
    communicator.read_command()

    if increase:
        indicator.intensity(indicator.intensity() + 5)
    else:
        indicator.intensity(indicator.intensity() - 5)

    if indicator.intensity() <= 0:
        increase = True
    elif indicator.intensity() >= 255:
        increase = False

    pyb.delay(5)
