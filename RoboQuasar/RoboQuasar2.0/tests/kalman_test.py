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
tmp36 = Sensor(0, ['temp'])
mcp9808 = Sensor(1, ['temp'])
builtin_accel = Sensor(2, ['x', 'y', 'z'])
gps = Sensor(3, ['lat', 'long', 'speed', 'heading', 'hdop'])
accel_gyro = Sensor(4, ['accel_x', 'accel_y', 'accel_z',
                        'gyro_x', 'gyro_y', 'gyro_z'])
compass = Sensor(5, ['heading'])
encoder = Sensor(6, ['counts'])
imu = Sensor(7, ['accel_x', 'accel_y', 'accel_z',
                 'gyro_x', 'gyro_y', 'gyro_z',
                 'yaw', 'pitch', 'roll',
                 'quat_w', 'quat_x', 'quat_y', 'quat_z'])

# encoder = Sensor(2, ['distance', 'delta'])

servo_steering = Command(0, 'position', (90, -90))
# servo_brakes = Command(1, 'position', (90, -90))
motor = Command(1, 'speed', (-255, 255))



# not seeing any data? try rebooting the board
# run basic_serial_test.py to make sure that data
# is coming in

start(use_handshake=False)

log_data = False
log = None

should_go = is_running()
if should_go == False:
    print("Hit enter to check if the robot is ready")
    print("Type R then enter to start the program")

    while should_go == False:
        value = input("> ")
        if value == "":
            print(is_running())
        if value.upper() == "R":
            should_go = True

k_filter = MainFilter(gps['lat'], gps['long'], 1, 0)


if log_data:
    log = Recorder(frequency=1.0, file_name='test')
    log.add_tracker(tmp36, "temperature C")
    log.add_tracker(mcp9808, "temperature C")
    log.add_tracker(accel_gyro, 'imu')
    log.add_tracker(compass, 'compass heading')
    log.add_tracker(encoder, 'encoder')
    log.add_tracker(builtin_accel, 'builtin accel')
    log.add_tracker(gps, 'gps')
    log.add_tracker(servo_steering, 'servo_steering')
    # log.add_tracker(servo_brakes, 'servo_brakes')
    log.add_tracker(motor, 'motor')
    log.end_init()

try:
    while True:
        # print(accel_gyro["accel_x"], accel_gyro["accel_y"], accel_gyro["accel_z"])
        # print(accel_gyro["gyro_x"], accel_gyro["gyro_y"], accel_gyro["gyro_z"])
        # print(compass["heading"])
        # print(encoder["counts"])
        #
        # print(builtin_accel["x"], builtin_accel["y"], builtin_accel["z"])
        # print(gps["lat"], gps["long"], gps["speed"], gps["heading"], gps["hdop"])
        # print(tmp36["temp"], mcp9808["temp"])
        print("is alive:", is_running())
        # w, x, y, z = imu['quat_w'], imu['quat_x'], imu['quat_y'], imu['quat_z']
        # heading = math.atan2(2 * w * z + x * y, 1 - 2 * (y ** 2 + z ** 2))
        (x, y, phi) = k_filter.update(gps["lat"], gps["long"], encoder["counts"], imu["accel_x"], imu["accel_y"], imu["gyro_y"], imu['yaw'] * math.pi / 180)
        print("%0.4f\t%0.4f\t%0.4f" % (x, y, phi))
        print(imu['yaw'] * math.pi / 180)

        if log_data:
            log.add_data(tmp36)
            log.add_data(mcp9808)
            log.add_data(accel_gyro)
            log.add_data(compass)
            log.add_data(encoder)
            log.add_data(builtin_accel)
            log.add_data(gps)
            log.add_data(servo_steering)
            # log.add_data(servo_brakes)
            log.end_row()

        time.sleep(0.05)
except:
    traceback.print_exc()
finally:
    stop()
