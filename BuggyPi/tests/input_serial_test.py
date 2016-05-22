import sys

sys.path.insert(0, "../")

from microcontroller.data import *
from microcontroller.dashboard import *


def angle_to_servo(angle):
    return


def main():
    leds = [Command(command_id, (0, 2)) for command_id in range(4)]
    # servo = Command(0, (-90, 90))

    start()

    try:
        while True:
            value = input("> ")
            if value == 's':
                print("Aborted by user")
                break
            if '1' in value:
                leds[0].set(2)
            if '2' in value:
                leds[1].set(2)
            if '3' in value:
                leds[2].set(2)
            if '4' in value:
                leds[3].set(2)

            # servo.set(angle_to_servo(
            #     math.atan2(1, -5.34 / 90 * joystick.mainStick.x)))

    except KeyboardInterrupt:
        stop()


if __name__ == '__main__':
    main()
