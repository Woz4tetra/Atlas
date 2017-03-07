# example of creating a joystick using buggy_joystick (uses pygame)
# use a pygame event reader to find the button mappings

from atlasbuggy.buggyjoystick import BuggyJoystick, get_platform


class ExampleJoystick(BuggyJoystick):
    def __init__(self):
        platform = get_platform()
        if platform == "mac":
            super(ExampleJoystick, self).__init__(
                ['left x', 'left y', 'right x', 'right y', 'ZL', 'ZR'],
                [0.3, -0.3, 0.0, 0.3, 0.3, 0.0],
                ['up', 'down', 'left', 'right', '+', '-', 'left stick', 'right stick',
                 'L', 'R', 'home', 'A', 'B', 'X', 'Y'],
            )
        else:
            super(ExampleJoystick, self).__init__(
                ['left x', 'left y', 'right x', 'right y', 'ZL', 'ZR'],
                [0.3, -0.3, 0.0, 0.3, 0.3, 0.0],
                ['A', 'B', 'X', 'Y', 'L', 'R', '-', '+', 'home', 'left stick', 'right stick'],
            )


if __name__ == '__main__':
    import time

    joystick = ExampleJoystick()

    while True:
        joystick.update()
        print(joystick)
        time.sleep(0.15)
