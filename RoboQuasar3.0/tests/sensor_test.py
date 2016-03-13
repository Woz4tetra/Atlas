"""
Written by Ben Warwick

sensor_test.py, written for RoboQuasar3.0
Version 3/10/2015
=========

A test of sensor communications. Prints all incoming sensor data
"""

import traceback
import sys
import time

sys.path.insert(0, '../')

from microcontroller.data import Sensor
from microcontroller.data import Command
from microcontroller.data import start, stop, is_running

from analyzers.logger import Recorder

# data type is specified by incoming packet
gps = Sensor(1, ['lat', 'long', 'heading'])
encoder = Sensor(2, 'counts')
imu = Sensor(3, ['accel_x', 'accel_y', 'yaw', 'compass'])

servo_steering = Command(0, 'position', (90, -90))

# servo_steering = Command(0, 'position', (90, -90))
# servo_brakes = Command(1, 'position', (90, -90))

# not seeing any data? try rebooting the board
# run basic_serial_test.py to make sure that data
# is coming in

start(use_handshake=False, check_status=False)

log_data = True
log = None

if log_data:
    log = Recorder(frequency=0.1)
    log.add_tracker(encoder, 'encoder')
    log.add_tracker(gps, 'gps')
    log.add_tracker(imu, 'imu')
    log.add_tracker(servo_steering, 'servo_steering')
    # log.add_tracker(servo_brakes, 'servo_brakes')
    log.end_init()

prev_status = is_running()

try:
    while True:
        if imu.recved_data():
            print(("%0.4f\t" * 4) % (imu["accel_x"], imu["accel_y"],
                                     imu["compass"], imu["yaw"]))
        if gps.recved_data():
            print(gps["lat"], gps["long"], gps["heading"])
        if encoder.recved_data():
            print(encoder["counts"])
            # time.sleep(0.25)

        if is_running() != prev_status:
            if is_running() == True:
                print("Connection made!")
            else:
                print("Connection lost...")
            prev_status = is_running()

        if log_data:
            log.add_data(encoder)
            log.add_data(imu)
            log.add_data(gps)
            log.add_data(servo_steering)
            # log.add_data(servo_brakes)
            log.end_row()

        time.sleep(0.005)
except KeyboardInterrupt:
    traceback.print_exc()
finally:
    stop()
