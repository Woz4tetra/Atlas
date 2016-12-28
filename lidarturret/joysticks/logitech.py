import sys

sys.path.insert(0, "../")

from atlasbuggy.buggyjoystick import BuggyJoystick


class Logitech(BuggyJoystick):
    def __init__(self):
        super(Logitech, self).__init__(
            ['left x', 'left y', 'right x', 'right y'],
            [0.3, 0.3, 0.0, 0.3, 0.3, 0.0],
            ['X', 'A', 'B', 'Y', 'L', 'R', 'ZL', 'ZR', 'back', 'left stick',
             'left stick', 'right stick'],
        )


if __name__ == '__main__':
    import time

    joystick = Logitech()

    while True:
        joystick.update()
        print(joystick)
        time.sleep(0.15)
