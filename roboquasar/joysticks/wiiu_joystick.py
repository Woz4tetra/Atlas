from atlasbuggy.buggy_joystick import BuggyJoystick


class WiiUJoystick(BuggyJoystick):
    def __init__(self, button_down_fn=None, button_up_fn=None,
                 axis_active_fn=None, axis_inactive_fn=None,
                 dpad_active_fn=None, dpad_inactive_fn=None):
        super(WiiUJoystick, self).__init__(
            ['left x', 'left y', 'ZL', 'right x', 'right y', 'ZR'],
            [0.3, 0.3, 0.0, 0.3, 0.3, 0.0],
            ['A', 'B', 'X', 'Y', 'L', 'R', '-', '+', '', 'left stick',
             'right stick'],
            button_down_fn, button_up_fn, axis_active_fn,
            axis_inactive_fn, dpad_active_fn, dpad_inactive_fn,
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
