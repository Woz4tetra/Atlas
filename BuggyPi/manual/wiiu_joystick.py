import sys

sys.path.insert(0, "../")

from manual.buggy_joystick import *

class WiiUJoystick(BuggyJoystick):
    def __init__(self):
        super(WiiUJoystick, self).__init__(
            ['left x', 'left y', 'right x', 'right y'],
            [0.2, 0.2, 0.2, 0.2],
            ['Y', 'B', 'A', 'X', 'L', 'R', 'ZL', 'ZR', '-', '+', 'joy L', 'joy R']
        )

if __name__ == '__main__':
    import time

    joystick = WiiUJoystick()
    joystick.start()

    try:
        while True:
            print(joystick)
            time.sleep(0.15)
    except KeyboardInterrupt:
        joystick.stop()
