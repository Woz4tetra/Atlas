import sys
import pygame
import os
import traceback

sys.path.insert(0, "../")

from microcontroller.data import *
from microcontroller.dashboard import *
from manual.wiiu_joystick import WiiUJoystick
# from manual.gc_joystick import GCjoystick


def stick_to_servo(x):
    return int(-math.atan2(x, 1) * 180 / (math.pi * 1.85))


def main():
##    leds = [Command(command_id, "led " + str(command_id), (0, 2)) for command_id in range(4)]
    servo = Command(4, 'servo', (-90, 90), use_repeats=True)
    motors = Command(5, 'motors', (-100, 100), use_repeats=True)

    counts = Sensor(0, 'encoder')
    gps = Sensor(1, 'gps', ['lat', 'long', 'altitude', 'found'])
    yaw = Sensor(2, 'imu', 'yaw')
    altitude = Sensor(3, 'altitude')
    checkpoint_num = 0

    pygame.init()
    pygame.joystick.init()
    joystick = WiiUJoystick()

    joystick.start()
    start(log_data=True)

    try:
        while is_running():
            if joystick.get_button('B'):
                print("Aborted by user")
                break

            servo.set(stick_to_servo(joystick.get_axis("left x")))
            value = -joystick.get_axis("left y")
            if value != 0:
                value = 1 * ((value > 0) - (value < 0))
            motors.set(int(value * 100))

            print("%0.4f, %5.0i, (%0.6f, %0.6f, %i), %i" % (yaw.get(), counts.get(),
                  gps.get('lat'), gps.get('long'), gps.get('found'), checkpoint_num), end='\r')

            if joystick.get_button('A'):  # TODO: switch to a event based system (becomes false after access)
                record('checkpoint reached!', checkpoint_num)
                checkpoint_num += 1
                while joystick.get_button('A'):  pass
    except:
        traceback.print_exc()
    finally:
        joystick.stop()
        stop()

if __name__ == '__main__':
    main()
