import traceback
import sys
import time

sys.path.insert(0, '../')

from board.data import Command
from board.data import start, stop, command_queue
from controller.gcjoystick import joystick_init

servo = Command(0, 'position', (-90, 90))

joystick = joystick_init()

start(use_handshake=False)

try:
    while True:
        joystick.update()
        servo.position = int(
            50 * -(joystick.triggers.L - joystick.triggers.R))
        print(servo.current_packet)
        # print(command_queue.queue)
        time.sleep(0.05)
except:
    traceback.print_exc()
    stop()
