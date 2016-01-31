import traceback
import sys

sys.path.insert(0, '../')

import time

from board.data import Sensor
from board.data import Command
from board.data import start, stop

from board.logger import Recorder

# data type is specified by incoming packet
tmp36 = Sensor(0, ['temp'])
mcp9808 = Sensor(1, ['temp'])
builtin_accel = Sensor(2, ['x', 'y', 'z'])
gps = Sensor(3, ['lat', 'long', 'speed', 'heading', 'hdop'])
accel_gyro = Sensor(4, ['accel_x', 'accel_y', 'accel_z',
                        'gyro_x', 'gyro_y', 'gyro_z'])
compass = Sensor(5, ['heading'])
encoder = Sensor(6, ['counts'])

# encoder = Sensor(2, ['distance', 'delta'])

servo_steering = Command(0, 'position', (90, -90))
# servo_brakes = Command(1, 'position', (90, -90))
motor = Command(1, 'speed', (-255, 255))

start(use_handshake=False)

servo_increase = True

time_stamp0 = 0

log_data = True
log = None

if log_data:
    log = Recorder()
    # log.add_tracker(tmp36, 'tmp36', "temperature C")
    # log.add_tracker(mcp9808, 'mcp9808', "temperature C")
    # log.add_tracker(imu, 'imu')
    # log.add_tracker(encoder, 'encoder')
    log.add_tracker(builtin_accel, 'builtin_accel')
    log.add_tracker(gps, 'gps')
    log.add_tracker(servo_steering, 'servo_steering')
    # log.add_tracker(servo_brakes, 'servo_brakes')
    log.add_tracker(motor, 'motor')
    log.end_init()

try:
    while True:
        print(accel_gyro.accel_x, accel_gyro.accel_y, accel_gyro.accel_z)
        print(accel_gyro.gyro_x, accel_gyro.gyro_y, accel_gyro.gyro_z)
        print(compass.heading)
        print(encoder.counts)

        # print(builtin_accel.x, builtin_accel.y, builtin_accel.z)
        # print(gps.lat, gps.long, gps.speed, gps.heading, gps.hdop)
        # print(tmp36.temp, mcp9808.temp)

        # print(servo_steering.position)

        # if servo_increase:
        #     servo_steering.position += 1
        # else:
        #     servo_steering.position -= 1
        #
        # if servo_steering.position == 90:
        #     servo_increase = False
        # elif servo_steering.position == -90:
        #     servo_increase = True
        #
        # motor.speed = servo_steering.position + 90

        if log_data:
            # time_stamp1 = int(time.time() - log.time0)
            # if time_stamp1 != time_stamp0:
            #     time_stamp0 = time_stamp1
            # log.add_data(tmp36)
            # log.add_data(mcp9808)
            # log.add_data(imu)
            # log.add_data(encoder)
            log.add_data(builtin_accel)
            log.add_data(gps)
            log.add_data(servo_steering)
            # log.add_data(servo_brakes)
            log.end_row()

        time.sleep(0.005)
except:
    traceback.print_exc()
    stop()
