"""
Written by Ben Warwick

Data recorder. Written for test day 1
Version 3/7/16
"""

import traceback
import sys
import time
import math

sys.path.insert(0, '../')

from board.data import Sensor
from board.data import Command
from board.data import start, stop, is_running
from controller.gcjoystick import joystick_init

from board.logger import Recorder

from board.filter import MainFilter

# data type is specified by incoming packet
gps = Sensor(1, ['lat_min', 'lat_sec',
                 'long_min', 'long_sec',
                 'speed', 'heading', 'hdop'])
encoder = Sensor(2, ['counts'])
imu = Sensor(3, ['accel_x', 'accel_y', 'accel_z',
                 'gyro_x', 'gyro_y', 'gyro_z',
                 'yaw', 'pitch', 'roll',
                 'quat_w', 'quat_x', 'quat_y', 'quat_z'])

servo_steering = Command(0, 'position', (90, -90))
# servo_brakes = Command(1, 'position', (90, -90))

joystick = joystick_init()

start(use_handshake=False)

log_data = False
log = None

time.sleep(0.5)

k_filter = MainFilter(gps['lat_sec'], gps['long_sec'], 1, 0, encoder['counts'])

if log_data:
    log = Recorder(frequency=0.01)
    log.add_tracker(imu, 'imu')
    log.add_tracker(encoder, 'encoder')
    log.add_tracker(gps, 'gps')
    log.add_tracker(servo_steering, 'servo_steering')
    # log.add_tracker(servo_brakes, 'servo_brakes')
    log.end_init()

try:
    while True:
        print("is alive:", is_running())
        x, y, phi = k_filter.update(gps["lat_sec"], gps["long_sec"],
                                    encoder["counts"], imu["accel_x"],
                                    imu["accel_y"],
                                    imu["gyro_z"], imu['yaw'] * math.pi / 180)
        print("%0.4f\t%0.4f\t%0.4f" % (x, y, phi))

        joystick.update()
        servo_steering["position"] = int(
            50 * (joystick.triggers.L - joystick.triggers.R)) - 25

        if log_data:
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
