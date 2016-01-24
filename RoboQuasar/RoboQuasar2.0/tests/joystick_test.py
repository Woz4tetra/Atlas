import traceback
import sys
import time

sys.path.insert(0, '../')

from board.data import Command
from board.data import start, stop
from controller.gcjoystick import joystick_init

servo = Command(0, 'position', (90, -90))

start(use_handshake=False)

joystick = joystick_init()

try:
    while True:
        joystick.update()

        servo.position = int(
                50 * (joystick.triggers.L - joystick.triggers.R))
        print(servo.position)
except:
    traceback.print_exc()
    stop()
