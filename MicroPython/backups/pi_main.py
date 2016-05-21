# main.py -- put your code here!

import pyb
from pyb import UART
from objects import *
from data import *
from comm import Communicator
from logger import Recorder

leds = [CommandLED(index, index + 1) for index in range(4)]
servo_steering = Servo(4, 1, start_pos=0)

communicator = Communicator(servo_steering)

while True:
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
