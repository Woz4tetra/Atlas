import traceback
import sys
import time

sys.path.insert(0, '../')

from board.data import Sensor
from board.data import Command
from board.data import start, stop, is_running

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
imu = Sensor(7, ['accel_x', 'accel_y', 'accel_z',
                 'gyro_x', 'gyro_y', 'gyro_z',
                 'quat_w', 'quat_x', 'quat_y', 'quat_z'])

# encoder = Sensor(2, ['distance', 'delta'])

servo_steering = Command(0, 'position', (90, -90))
# servo_brakes = Command(1, 'position', (90, -90))
motor = Command(1, 'speed', (-255, 255))

# not seeing any data? try rebooting the board
# run basic_serial_test.py to make sure that data
# is coming in

start(use_handshake=False)

servo_increase = True

log_data = False
log = None

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
        # print("%0.4f\t%0.4f\t%0.4f" % (accel_gyro["accel_x"], accel_gyro["accel_y"], accel_gyro["accel_z"]))
        # print("%0.4f\t%0.4f\t%0.4f" % (accel_gyro["gyro_x"], accel_gyro["gyro_y"], accel_gyro["gyro_z"]))
        # print(compass["heading"])
        print("%0.4f\t%0.4f\t%0.4f" % (imu["accel_x"], imu["accel_y"], imu["accel_z"]))
        print("%0.4f\t%0.4f\t%0.4f" % (imu["gyro_x"], imu["gyro_y"], imu["gyro_z"]))
        print("%0.4f\t%0.4f\t%0.4f\t%0.4f" % (imu["quat_w"], imu["quat_x"], imu["quat_y"], imu["quat_z"]))

        # print(builtin_accel["x"], builtin_accel["y"], builtin_accel["z"])
        # print(gps["lat"], gps["long"], gps["speed"], gps["heading"], gps["hdop"])
        # print(tmp36["temp"], mcp9808["temp"])
        print("is alive:", is_running())

        # time.sleep(1)
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

        time.sleep(0.005)
except:
    traceback.print_exc()
finally:
    stop()
