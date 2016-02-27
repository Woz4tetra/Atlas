# A file we would run should go out to roll to collect data again
# (assuming we have the buggy remote controlled)

import sys
import traceback

sys.path.insert(0, '../')

from board import logger
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

log = logger.Recorder()

try:
    while True:
        print((imu._data))
        print((gps._data))
        print((encoder._data))

        log.add_data("imu", imu)
        log.add_data("gps", gps)
        log.add_data("encoder", encoder)

except:
    traceback.print_exc()
    comm.exit_flag = True
