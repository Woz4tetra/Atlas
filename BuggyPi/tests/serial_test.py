import sys

sys.path.insert(0, "../")

from microcontroller.data import *
from microcontroller.dashboard import *

from manual.buggy_joystick import joystick_init
from manual.joysticks import WiiUJoystick


def angle_to_servo(angle):
    return


def main():
    leds = [Command(command_id, (0, 2)) for command_id in range(4)]
    servo = Command(0, (-90, 90))

    joystick = joystick_init(WiiUJoystick)

    start()

    try:
        while True:
            if joystick.buttons.B:
                print("Aborted by user")
                break
            elif joystick.buttons.A:
                leds[0].set(2)
                while joystick.buttons.A: pass
            elif joystick.buttons.X:
                leds[1].set(2)
                while joystick.buttons.X: pass
            elif joystick.buttons.Y:
                leds[2].set(2)
                while joystick.buttons.Y: pass

            servo.set(angle_to_servo(
                math.atan2(1, -5.34 / 90 * joystick.mainStick.x)))

            time.sleep(0.005)
    except KeyboardInterrupt:
        stop()
        joystick.stop()

if __name__ == '__main__':
    main()
