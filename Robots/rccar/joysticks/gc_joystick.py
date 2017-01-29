from autobuggy.buggy_joystick import *


class GCjoystick(BuggyJoystick):
    def __init__(self):
        super(GCjoystick, self).__init__(
            ['main x', 'main y', 'c y', 'c x', 'L', 'R'],
            [0.15, 0.15, 0.15, 0.15, 0.1, 0.1],
            ['X', 'A', 'B', 'Y', 'L', 'R', '', 'Z', '', 'start']
        )


if __name__ == '__main__':
    import time

    joystick = GCjoystick()
    joystick.start()

    try:
        while True:
            print(joystick)
            time.sleep(0.15)
    except KeyboardInterrupt:
        joystick.stop()