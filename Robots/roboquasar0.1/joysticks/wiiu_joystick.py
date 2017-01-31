from atlasbuggy.buggyjoystick import BuggyJoystick
from atlasbuggy import project

class WiiUJoystick(BuggyJoystick):
    def __init__(self):
        platform = project.get_platform()
        if platform == "mac":
            super(WiiUJoystick, self).__init__(
                ['left x', 'left y', 'right x', 'right y', 'ZL', 'ZR'],
                [0.3, -0.3, 0.0, 0.3, 0.3, 0.0],
                ['up', 'down', 'left', 'right', '+', '-', 'left stick', 'right stick', 'L', 'R', 'home', 'A', 'B', 'X', 'Y'],
            )
        else:
            super(WiiUJoystick, self).__init__(
                ['left x', 'left y', 'right x', 'right y', 'ZL', 'ZR'],
                [0.3, -0.3, 0.0, 0.3, 0.3, 0.0],
                ['A', 'B', 'X', 'Y', 'L', 'R', '-', '+', 'home', 'left stick', 'right stick'],
            )


if __name__ == '__main__':
    import time

    joystick = WiiUJoystick()

    while True:
        joystick.update()
        print(joystick)
        time.sleep(0.15)
