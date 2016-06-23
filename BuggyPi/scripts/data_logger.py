import sys

sys.path.insert(0, "../")

from microcontroller.data import *
from microcontroller.comm import *
from manual.wiiu_joystick import WiiUJoystick


# from manual.gc_joystick import GCjoystick

def stick_to_servo(x):
    return int(-math.atan2(x, 1) * 180 / (math.pi * 1.85))

def button_dn(button, params):
    global checkpoint_num, gps
    if button == 'B':
        print("Aborted by user")
    elif button == 'A':
        communicator.record('checkpoint', num=checkpoint_num, coord=(gps.get("long"), gps.get("lat")))
        checkpoint_num += 1

def joy_changed(axis, value, params):
    global servo, motors, joystick
    if axis == "left x":
        servo.set(stick_to_servo(value))
    if axis == "left y":
        value = -joystick.get_axis("left y")
        if value != 0:
            value = 1 * ((value > 0) - (value < 0))
        motors.set(int(value * 100))

counts = Sensor(0, 'encoder', 'counts')
gps = Sensor(1, 'gps', ['lat', 'long', 'altitude', 'found'])
yaw = Sensor(2, 'imu', 'yaw')
altitude = Sensor(3, 'altitude', 'altitude')
checkpoint_num = 0

sensor_pool = SensorPool(counts, gps, yaw, altitude)
communicator = Communicator(sensor_pool)

if not communicator.initialized:
    quit()

leds = [Command(command_id, "led " + str(command_id), (0, 2), communicator)
        for command_id in range(4)]
servo = Command(4, 'servo', (-90, 90), communicator)
motors = Command(5, 'motors', (-100, 100), communicator)

joystick = WiiUJoystick(button_down_fn=button_dn,
                        axis_active_fn=joy_changed)


def main():
    joystick.start()
    communicator.start()

    try:
        while True:
            if yaw.received():
                print(yaw)
            if gps.received():
                print(gps)
            if counts.received():
                print(counts)
##            if altitude.received():
##                print(altitude)
            time.sleep(0.05)

    except:
        traceback.print_exc()
    finally:
        joystick.stop()
        communicator.stop()


if __name__ == '__main__':
    main()
