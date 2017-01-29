from atlasbuggy.buggyjoystick import BuggyJoystick


class WiiUJoystick(BuggyJoystick):
    def __init__(self):
        super(WiiUJoystick, self).__init__(
            ['left x', 'left y', 'right x', 'right y', 'ZL', 'ZR'],
            [0.3, -0.3, 0.0, 0.3, 0.3, 0.0],
            ['', '', '', '', '+', '-', 'left stick', 'right stick', 'L', 'R', 'home', 'A', 'B', 'X', 'Y'],
        )


if __name__ == '__main__':
    import time

    joystick = WiiUJoystick()

    while True:
        joystick.update()
        print(joystick)
        time.sleep(0.15)
