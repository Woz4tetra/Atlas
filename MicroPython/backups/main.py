# main.py -- put your code here!

import pyb
from pyb import UART
from objects import *
from data import *
from comm import Communicator
from logger import Recorder

gps = GPS(1)
encoder = HallEncoder(2, "X7")
imu = IMU(3, 2)

servo_steering = Servo(0, 1, start_pos=-23)

gps_indicator = pyb.LED(3)
pyb.LED(1).on()

new_data = False


def pps_callback(line):
    global new_data, gps_indicator
    new_data = True
    gps_indicator.toggle()


uart = UART(6, 9600, read_buf_len=1000)
pps_pin = pyb.Pin.board.X8
extint = pyb.ExtInt(pps_pin, pyb.ExtInt.IRQ_FALLING,
                    pyb.Pin.PULL_UP, pps_callback)

command_pool = CommandPool(
    servo_steering,
)

communicator = Communicator(command_pool)

while True:
    if new_data:
        while uart.any():
            gps.update(chr(uart.readchar()))
        communicator.write_packet(gps)
        new_data = False

    communicator.write_packet(imu)
    if encoder.recved_data():
        communicator.write_packet(encoder)

    communicator.read_command()

    if communicator.reset_code:
        gps.reset()
        imu.reset()
        encoder.reset()
        servo_steering.reset()

        if communicator.reset_code == "R":
            communicator.write_packet(gps)
            pyb.delay(1)

            communicator.write_packet(imu)
            pyb.delay(1)

            communicator.write_packet(encoder)

        communicator.reset_code = None

    pyb.delay(1)
