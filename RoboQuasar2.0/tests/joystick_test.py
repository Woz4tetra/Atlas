import traceback
import sys
import time

sys.path.insert(0, '../')

from board.data import Sensor
from board.data import Command
from board.data import start, stop, is_running
from controller.gcjoystick import joystick_init

servo_control = Command(0, ['servo_num', (0, 15), 'position', (90, -90)])

joystick = joystick_init()

start(use_handshake=False)

try:
    while True:
        joystick.update()
        servo_control.set(servo_num=0, position=int(
            50 * -(joystick.triggers.L - joystick.triggers.R)))
        servo_control.set(servo_num=1, position=int(-(joystick.cStick.y * .85 / 90)))

        time.sleep(0.5)
except:
    traceback.print_exc()
    stop()
