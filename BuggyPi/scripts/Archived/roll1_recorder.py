# mimics the script run during RoboQuasar's first coursewalk

import sys
import traceback

sys.path.insert(0, '../')

from board import logger
from . import gc_joystick
from board import comm
from board.data import *
from board.arduino_objects import *

imu = IMU(0)
gps = GPS(1)
encoder = Encoder(2)

servo = Servo(0)
led13 = Led13(1)

sensor_data = SensorPool(imu, gps, encoder)
command_queue = CommandQueue(servo=servo, led13=led13)

communicator = comm.Communicator(115200, command_queue, sensor_data)
communicator.start()

servo_value = 0
led13_value = True

joystick = gc_joystick.init()

log = logger.Recorder()

def joystick_angle(position):
    if math.sqrt(position[0] ** 2 + position[1] ** 2) > 0.6:
        return -int(
            math.degrees(math.atan2(position[1], position[0])) * 125 / 180)
    else:
        return -1


try:
    while joystick.done is False:
        joystick.update()

        servo_value = joystick_angle(joystick.mainStick)

        print((imu._data))
        print((gps._data))
        print((encoder._data))

        log.add_data("imu", imu)
        log.add_data("gps", gps)
        log.add_data("encoder", encoder)

        if 0 <= servo_value <= 125:
            command_queue.put(servo, servo_value)
            # command_queue.put(led13, led13_value)

except:
    traceback.print_exc()
    comm.exit_flag = True
