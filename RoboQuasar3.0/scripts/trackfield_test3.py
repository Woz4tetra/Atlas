"""
Written by Ben Warwick

trackfield_test1.py, written for RoboQuasar3.0
Version 3/10/2015
=========

Data recorder. Written for test day 3
"""

import traceback
import sys
import time
import math

sys.path.insert(0, '../')

from microcontroller.data import Sensor
from microcontroller.data import Command
from microcontroller.data import start, stop, is_running
from analyzers.logger import Recorder

from controllers.joystick import joystick_init
from controllers.servo_map import *

script_options = dict(
    log_data=False,
    enable_joystick=True,
    print_data=True
)

# data type is specified by incoming packet
gps = Sensor(1, ['lat', 'long', 'heading', 'found'])
encoder = Sensor(2, 'counts')
imu = Sensor(3, ['accel_x', 'accel_y', 'yaw', 'compass'])

servo_steering = Command(0, 'position', (90, -90))
# servo_brakes = Command(1, 'position', (90, -90))

start(use_handshake=False)

if script_options['enable_joystick']:
    joystick = joystick_init()

log = None

prev_status = not is_running()

time.sleep(0.5)

if script_options['log_data']:
    log = Recorder(frequency=0.01)
    log.add_tracker(imu, 'imu')
    log.add_tracker(encoder, 'encoder')
    log.add_tracker(gps, 'gps')
    log.add_tracker(servo_steering, 'servo_steering')
    # log.add_tracker(servo_brakes, 'servo_brakes')
    log.end_init()

try:
    while True:
        if script_options['print_data']:
            # if imu.received():
            #     print(("%0.4f\t" * 4) % (
            #         imu["accel_x"], imu["accel_y"],
            #         imu["compass"], imu["yaw"]))
            if gps.received():
                print(gps["lat"], gps["long"], gps["heading"], gps["found"])
            if encoder.received():
                print(encoder["counts"])
                # time.sleep(0.25)

        if is_running() != prev_status:
            if is_running():
                print("Connection made!")
            else:
                print("Connection lost...")
            prev_status = is_running()

        if script_options['enable_joystick']:
            # servo_steering["position"] = int(
            #     50 * (joystick.triggers.L - joystick.triggers.R)) - 23
            servo_steering["position"] = \
                state_to_servo([0, 0, 0],
                               [1, 5.34 / 90 * joystick.mainStick.x])

        if script_options['log_data']:
            log.add_data(imu)
            log.add_data(encoder)
            log.add_data(gps)
            log.add_data(servo_steering)
            # log.add_data(servo_brakes)
            log.end_row()

        time.sleep(0.005)
except:
    traceback.print_exc()
finally:
    stop()
    joystick.stop()
