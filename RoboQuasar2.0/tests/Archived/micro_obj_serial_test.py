import time
import traceback
import sys

sys.path.insert(0, '../')

from board import comm
from board import logger
from board.micropy_objects import *
from board.data import *

tmp36 = TMP36(0)
mcp9808 = MCP9808(1)
builtin_accel = BuiltinAccel(2)
gps = GPS(3)
imu = MPU6050(4)
compass = HMC5883L(5)
# encoder_right = RotaryEncoder(6)

servo1 = Servo(0, -90)

sensor_data = SensorPool(
        tmp36,
        mcp9808,
        builtin_accel,
        gps,
        imu,
        compass
)
command_queue = CommandQueue()

communicator = comm.Communicator(115200, command_queue, sensor_data,
                                 use_handshake=False)
communicator.start()

servo_increase = True

log_data = False
log = None
if log_data:
    log = logger.Recorder()

    log.add_tracker(tmp36, 'tmp36', "temperature C")
    log.add_tracker(mcp9808, 'mcp9808', "temperature C")
    log.add_tracker(builtin_accel, 'builtin_accel', "accel x", "accel y",
                   "accel z")
    log.add_tracker(gps, 'gps', "lat deg", "long deg", "lat min", "long deg",
                   "speed", "hdop", "heading")
    log.add_tracker(servo1, 'servo1', "degrees")
    log.end_init()

time_stamp0 = 0

try:
    while True:
        if servo_increase:
            servo1.offset(1)
        else:
            servo1.offset(-1)

        if servo1.degrees == 90 or servo1.degrees == -90:
            servo_increase = not servo_increase

        command_queue.put(servo1)

        time.sleep(0.005)

        # print(tmp36)
        # print(mcp9808)
        # print(builtin_accel)
        # print(gps)
        # print(servo1)
        # print(imu)
        print(int(compass.heading / 2) - 90)
        servo1.set(int(compass.heading / 2) - 90)
        command_queue.put(servo1)
        # print(encoder_right)

        if log_data:
            time_stamp1 = int(time.time() - log.time0)
            if time_stamp1 != time_stamp0:
                time_stamp0 = time_stamp1
                log.add_data(tmp36)
                log.add_data(mcp9808)
                log.add_data(builtin_accel)
                log.add_data(gps)
                log.add_data(servo1)
                log.end_row()
except:
    if log_data:
        log.close()
    traceback.print_exc()
    comm.exit_flag = True
