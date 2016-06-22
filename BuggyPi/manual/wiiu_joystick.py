import sys

sys.path.insert(0, "../")

from manual.buggy_joystick import *


class WiiUJoystick(BuggyJoystick):
    def __init__(self, button_down_fn=None, button_up_fn=None,
                 axis_active_fn=None, axis_inactive_fn=None, joy_hat_fn=None,
                 fn_params=None):
        super(WiiUJoystick, self).__init__(
            ['left x', 'left y', 'right x', 'right y'],
            [0.2, 0.2, 0.2, 0.2],
            ['Y', 'B', 'A', 'X', 'L', 'R', 'ZL', 'ZR', '-', '+', 'joy L',
             'joy R'],
            button_down_fn=button_down_fn,
            button_up_fn=button_up_fn,
            axis_active_fn=axis_active_fn,
            axis_inactive_fn=axis_inactive_fn,
            joy_hat_fn=joy_hat_fn,
            fn_params=fn_params
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
