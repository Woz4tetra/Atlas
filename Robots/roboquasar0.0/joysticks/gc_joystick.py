
from atlasbuggy.buggy_joystick import BuggyJoystick


class GCjoystick(BuggyJoystick):
    def __init__(self, button_down_fn=None, button_up_fn=None,
                 axis_active_fn=None, axis_inactive_fn=None, dpad_active_fn=None,
                 fn_params=None):
        super(GCjoystick, self).__init__(
            ['left x', 'left y', 'right y', 'right x', 'L', 'R'],
            [0.3, 0.3, 0.3, 0.3, 0.3, 0.3],
            ['X', 'A', 'B', 'Y', 'L', 'R', '', 'Z', '', 'start', '', '', '', '', '', '', ''],
            button_down_fn, button_up_fn, axis_active_fn,
            axis_inactive_fn, dpad_active_fn, fn_params
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
