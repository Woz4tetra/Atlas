if __name__ == '__main__':
    import sys

    sys.path.insert("../")

from atlasbuggy.joysticks.buggyjoystick import BuggyJoystick, get_platform


class WiiUJoystick(BuggyJoystick):
    def __init__(self):
        platform = get_platform()
        deadzones = [0.1, -0.1, 0.0, 0.01, 0.01, 0.0]
        if platform == "mac":
            super(WiiUJoystick, self).__init__(
                ['left x', 'left y', 'right x', 'right y', 'ZL', 'ZR'],
                deadzones,
                ['up', 'down', 'left', 'right', '+', '-', 'left stick', 'right stick', 'L', 'R', 'home', 'A', 'B', 'X',
                 'Y'],
            )
        else:
            super(WiiUJoystick, self).__init__(
                ['left x', 'left y', 'right x', 'right y', 'ZL', 'ZR'],
                deadzones,
                ['A', 'B', 'X', 'Y', 'L', 'R', '-', '+', 'home', 'left stick', 'right stick'],
            )


if __name__ == '__main__':
    import time

    joystick = WiiUJoystick()

    while True:
        joystick.update()
        print(joystick)
        time.sleep(0.15)
