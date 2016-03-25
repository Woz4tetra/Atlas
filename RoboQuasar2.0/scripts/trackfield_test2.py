"""
Written by Ben Warwick

Data recorder. Written for test day 2
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
from board.logger import Recorder
from board.filter import StateFilter
from board.interpreter import Interpreter

from controller.servo_control import servo_value

from map.map_parser import Binder

# data type is specified by incoming packet
gps = Sensor(1, ['lat', 'long'])
encoder = Sensor(2, ['counts'])
imu = Sensor(3, ['accel_x', 'accel_y', 'gyro_z', 'yaw', 'compass'])

servo_steering = Command(0, 'position', (90, -90))
# servo_brakes = Command(1, 'position', (90, -90))

start(use_handshake=False)

log_data = True
log = None

time.sleep(0.5)

def get_shift_angle():
    # make sure RoboQuasar is staying still!!!
    time0 = time.time()
    compass_sum = 0
    data_count = 0
    while (time.time() - time0) < 10:
        compass_sum += imu['compass']
        data_count += 1
    return compass_sum / data_count

k_filter = StateFilter()
interpreter = Interpreter(encoder['counts'], gps['lat'], gps['long'], imu['compass'])

binder = Binder(
    "Mon Mar  7 17;54;59 2016 GPS Map.csv",
    get_shift_angle(), gps['lat'], gps['long']
)

time0 = time.time()

def get_state():
    global time0
    gps_x, gps_y, enc_counts, gyro_z, yaw = interpreter.convert(
        gps["lat"], gps["long"], encoder["counts"], imu["gyro_z"], imu['yaw'])
    dt = time.time() - time0
    time0 = time.time()
    return k_filter.update(
        gps_x, gps_y, enc_counts, imu["accel_x"], imu["accel_y"], gyro_z, yaw, dt)

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
        # print("is alive:", is_running(), "\r")
        x, y, heading = get_state()
        print("%0.4f\t%0.4f\t%0.4f\r" % (x, y, heading))

        goal_x, goal_y = binder.bind((x, y))
        servo_steering["position"] = servo_value((x, y, heading), (goal_x, goal_y))

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