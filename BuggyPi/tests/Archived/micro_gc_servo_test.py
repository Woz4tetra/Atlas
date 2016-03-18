import time
import traceback
import sys

sys.path.insert(0, '../')
print("something")


from board import comm
from board import logger
from board.micropy_objects import *
from board.data import *

from controller import gcjoystick

# tmp36 = TMP36(0)
# mcp9808 = MCP9808(1)
# builtin_accel = BuiltinAccel(2)
# gps = GPS(3)

servo1 = Servo(0, 0)

sensor_data = SensorPool(
        # tmp36,
        # mcp9808,
        # builtin_accel,
        # gps
)
command_queue = CommandQueue()

communicator = comm.Communicator(115200, command_queue, sensor_data,
                                 use_handshake=False)
communicator.start()

log_data = False
log = None
if log_data:
    log = logger.Recorder()

    # log.add_sensor(tmp36, 'tmp36', "temperature C")
    # log.add_sensor(mcp9808, 'mcp9808', "temperature C")
    # log.add_sensor(builtin_accel, 'builtin_accel', "accel x", "accel y",
    #                "accel z")
    # log.add_sensor(gps, 'gps', "lat deg", "long deg", "lat min", "long deg",
    #                "speed", "hdop", "heading")
    log.add_tracker(servo1, 'servo1', "degrees")
    log.end_init()

joystick = gcjoystick.init()

time_stamp0 = 0
servo_increase = True

try:
    while True:
        joystick.update()
        servo_val = int(
            30 * (joystick.triggers.L - joystick.triggers.R))  # * 50
        print(servo_val)
        servo1.set(int(servo_val))
        command_queue.put(servo1)

        time.sleep(0.01)

        # if servo_increase:
        #     servo1.offset(5)
        # else:
        #     servo1.offset(-5)
        #
        # if servo1.degrees >= 60 or servo1.degrees <= -60:
        #     servo_increase = not servo_increase
        #
        # command_queue.put(servo1)
        # servo1.set(5)
        # command_queue.put(servo1)
        # print(servo1)
        # time.sleep(2)
        #
        # servo1.set(-25)
        # command_queue.put(servo1)
        # print(servo1)
        # time.sleep(2)

        # print(tmp36)
        # print(mcp9808)
        # print(builtin_accel)
        # print(gps)


        if log_data:
            time_stamp1 = int(time.time() - log.time0)
            if time_stamp1 != time_stamp0:
                time_stamp0 = time_stamp1
                # log.add_data(tmp36)
                # log.add_data(mcp9808)
                # log.add_data(builtin_accel)
                # log.add_data(gps)
                log.add_data(servo1)
                log.end_row()
except:
    if log_data:
        log.close()
    traceback.print_exc()
    comm.exit_flag = True
