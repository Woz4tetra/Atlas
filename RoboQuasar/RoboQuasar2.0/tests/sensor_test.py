import traceback
import sys
import time

sys.path.insert(0, '../')

from board.data import Sensor
from board.data import Command
from board.data import start, stop, is_running

from board.logger import Recorder

# data type is specified by incoming packet
gps = Sensor(1, ['lat', 'long', 'speed', 'heading', 'hdop'])
encoder = Sensor(2, ['counts'])
imu = Sensor(3, ['accel_x', 'accel_y', 'accel_z',
                 'gyro_x', 'gyro_y', 'gyro_z',
                 'yaw', 'pitch', 'roll',
                 'quat_w', 'quat_x', 'quat_y', 'quat_z'])

servo_steering = Command(0, 'position', (90, -90))
servo_brakes = Command(1, 'position', (90, -90))

# not seeing any data? try rebooting the board
# run basic_serial_test.py to make sure that data
# is coming in

start(use_handshake=False)

log_data = False
log = None

if log_data:
    log = Recorder(frequency=1.0, file_name='test')
    log.add_tracker(encoder, 'encoder')
    log.add_tracker(gps, 'gps')
    log.add_tracker(servo_steering, 'servo_steering')
    log.add_tracker(servo_brakes, 'servo_brakes')
    log.end_init()

try:
    while True:
        print("%0.4f\t%0.4f\t%0.4f" % (imu["accel_x"], imu["accel_y"], imu["accel_z"]))
        print("%0.4f\t%0.4f\t%0.4f" % (imu["gyro_x"], imu["gyro_y"], imu["gyro_z"]))
        print("%0.4f\t%0.4f\t%0.4f" % (imu["roll"], imu["pitch"], imu["yaw"]))
        print("%0.4f\t%0.4f\t%0.4f\t%0.4f" % (imu["quat_w"], imu["quat_x"], imu["quat_y"], imu["quat_z"]))

        print(gps["lat"], gps["long"], gps["speed"], gps["heading"], gps["hdop"])
        print("is alive:", is_running())

        if log_data:
            log.add_data(encoder)
            log.add_data(imu)
            log.add_data(gps)
            log.add_data(servo_steering)
            log.add_data(servo_brakes)
            log.end_row()

        time.sleep(0.005)
except:
    traceback.print_exc()
finally:
    stop()
