import traceback
import sys
import time
import math

sys.path.insert(0, '../')

from board.data import Sensor
from board.data import Command
from board.data import start, stop, is_running

from board.logger import Recorder

from controller.interpreter import MainFilter

# data type is specified by incoming packet
gps = Sensor(1, ['lat', 'long', 'speed', 'heading', 'hdop'])
encoder = Sensor(2, ['counts'])
imu = Sensor(3, ['accel_x', 'accel_y', 'accel_z',
                 'gyro_x', 'gyro_y', 'gyro_z',
                 'yaw', 'pitch', 'roll',
                 'quat_w', 'quat_x', 'quat_y', 'quat_z'])

start(use_handshake=False, check_status=True)

log_data = False
log = None

k_filter = MainFilter(gps['lat'], gps['long'], 1, 0)


if log_data:
    log = Recorder(frequency=1.0, file_name='test')
    log.add_tracker(imu, 'imu')
    log.add_tracker(encoder, 'encoder')
    log.add_tracker(gps, 'gps')
    log.add_tracker(servo_steering, 'servo_steering')
    log.add_tracker(servo_brakes, 'servo_brakes')
    log.end_init()

try:
    while True:
        print("is alive:", is_running())
        (x, y, phi) = k_filter.update(gps["lat"], gps["long"], encoder["counts"], imu["accel_x"], imu["accel_y"], imu["gyro_y"], imu['yaw'] * math.pi / 180)
        print("%0.4f\t%0.4f\t%0.4f" % (x, y, phi))
        print(imu['yaw'] * math.pi / 180)

        if log_data:
            log.add_data(imu)
            log.add_data(encoder)
            log.add_data(gps)
            log.add_data(servo_steering)
            log.add_data(servo_brakes)
            log.end_row()

        time.sleep(0.05)
except:
    traceback.print_exc()
finally:
    stop()
