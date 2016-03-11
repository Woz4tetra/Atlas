"""
Written by Ben Warwick

joystick_test.py, written for RoboQuasar3.0
Version 3/10/2015
=========

A test of servo communications with command line and joystick input
"""

import traceback
import sys
import time

sys.path.insert(0, '../')

from microcontroller.data import Sensor
from microcontroller.data import Command
from microcontroller.data import start, stop, is_running
# from controller.gcjoystick import joystick_init

servo = Command(0, 'position', (90, -90))

# joystick = joystick_init()

start(use_handshake=False)

try:
    while True:
        # joystick.update()
        # servo["position"] = int(
        #     50 * -(joystick.triggers.L - joystick.triggers.R)))

        servo["position"] = int(input("servo: "))

        time.sleep(0.005)
except:
    traceback.print_exc()
    stop()
