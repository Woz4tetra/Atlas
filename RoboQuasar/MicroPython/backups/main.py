# main.py -- put your code here!

import pyb
from pyb import UART
from objects import *
from data import *
from comm import Communicator

tmp36 = TMP36(0, pyb.Pin.board.Y12)
mcp9808 = MCP9808(1, 1)
accel = BuiltInAccel(2)
gps = GPS(3)
accel_gyro = AccelGyro(4, 1)
compass = Compass(5, 1)
encoder = HallEncoder(6, "X7")
imu = IMU(7, 2)

servo1 = Servo(0, 1)
motor_a = Motor(1, 'X2', 'X3')

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
        tmp36,
        mcp9808,
        accel,
        gps,
        compass,
        accel_gyro,
	    encoder,
        imu
)
command_pool = CommandPool(
        servo1,
        # motor_a
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
