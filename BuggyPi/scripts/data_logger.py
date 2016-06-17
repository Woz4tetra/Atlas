import sys

sys.path.insert(0, "../")

from microcontroller.data import *
from microcontroller.dashboard import *
from manual.wiiu_joystick import WiiUJoystick
##from manual.gc_joystick import GCjoystick


def stick_to_servo(x):
    return int(-math.atan2(x, 1) * 180 / (math.pi * 1.85))


def main():
    leds = [Command(command_id, "led " + str(command_id), (0, 2)) for command_id in range(4)]
    servo = Command(4, 'servo', (-90, 90))
    motors = Command(5, 'motors', (-100, 100))

    counts = Sensor(0, 'encoder')
    gps = Sensor(1, 'gps', ['lat', 'long', 'found'])
    yaw = Sensor(2, 'imu')
    altitude = Sensor(3, 'altitude')
    checkpoint_num = 0

    joystick = WiiUJoystick()

    joystick.start()
    start(log_data=False, soft_reboot=False)

    try:
        while True:
            if joystick.get_button('B'):
                print("Aborted by user")
                break

            servo.set(stick_to_servo(joystick.get_axis("left x")))
            value = -joystick.get_axis("left y")
            if value != 0 and abs(value) < 0.8:
                value = 0.8 * ((value > 0) - (value < 0))
            motors.set(int(value * 100))

##            if joystick.dpad[1] == 1:
##                leds[0].set(1)
##            else:
##                leds[0].set(0)
##
##            if joystick.dpad[1] == -1:
##                leds[1].set(1)
##            else:
##                leds[1].set(0)
##
##            if joystick.dpad[0] == 1:
##                leds[2].set(1)
##            else:
##                leds[2].set(1)
##
##            if joystick.dpad[0] == -1:
##                leds[3].set(1)
##            else:
##                leds[3].set(0)
            print("%0.4f, %5.0i, (%0.6f, %0.6f, %i), motors: %3.0i, servo: %3.0i, %3.6fm           " % (yaw.get(), counts.get(),
                  gps.get('lat'), gps.get('long'), gps.get('found'), motors.get(), servo.get(), altitude.get()), end='\r')

            if joystick.get_button('A'):  # TODO: switch to a event based system (becomes false after access)
                record('checkpoint reached!', checkpoint_num)
                checkpoint_num += 1
    except KeyboardInterrupt:
        joystick.stop()
        stop()
        print("\n------------\n\n")


if __name__ == '__main__':
    main()
