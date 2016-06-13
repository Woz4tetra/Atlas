import sys

sys.path.insert(0, "../")

from microcontroller.data import *
from microcontroller.dashboard import *
# from manual.wiiu_joystick import WiiUJoystick
from manual.gc_joystick import GCjoystick


def stick_to_servo(x):
    return int(-math.atan2(x, 1) * 180 / (math.pi * 1.85))


def main():
    leds = [Command(command_id, "led " + str(command_id), (0, 2)) for command_id in range(4)]
    servo = Command(4, 'servo', (-90, 90))
    motors = Command(5, 'motors', (-100, 100))

    encoder = Sensor(0, 'encoder', 'counts')
    imu = Sensor(2, 'imu', 'yaw')

    joystick = GCjoystick()

    joystick.start()
    start(log_data=True)

    try:
        while True:
            if joystick.get_button('B'):
                print("Aborted by user")
                break

            servo.set(stick_to_servo(joystick.get_axis("main x")))
            motors.set(int(-joystick.get_axis("c y") * 100))

            if joystick.dpad[1] == 1:
                leds[0].set(1)
            else:
                leds[0].set(0)

            if joystick.dpad[1] == -1:
                leds[1].set(1)
            else:
                leds[1].set(0)

            if joystick.dpad[0] == 1:
                leds[2].set(1)
            else:
                leds[2].set(1)

            if joystick.dpad[0] == -1:
                leds[3].set(1)
            else:
                leds[3].set(0)
            print("%0.4f, %5.0i" % (imu.get('yaw'), encoder.get('counts')),
                  end='\r')

            record('something', leds)

    except KeyboardInterrupt:
        stop()


if __name__ == '__main__':
    main()
