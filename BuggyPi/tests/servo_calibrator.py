import sys

sys.path.insert(0, "../")

from microcontroller.data import *
from microcontroller.comm import *
from manual.wiiu_joystick import WiiUJoystick

left_angle = 0.81096
right_angle = -0.53719
left_value = 35
right_value = -25


def servo_to_angle(servo_value):
    return ((left_angle - right_angle) /
            (left_value - right_value) *
            (servo_value - right_value) + right_angle)


def angle_to_servo(angle):
    return int(((left_value - right_value) /
                (left_angle - right_angle) *
                (angle - right_angle) + right_value))


def joy_changed(axis, value, params):
    global servo, joystick
    if axis == "left x":
        angle = math.atan2(value, 1)
        servo.set(angle_to_servo(angle))
        print(servo.get(), servo_to_angle(angle), angle_to_servo(angle))
        time.sleep(0.05)


sensor_pool = SensorPool()
communicator = Communicator(sensor_pool, address='/dev/ttyAMA0', log_data=False)
if not communicator.initialized:
    raise Exception("Communicator not initialized...")

# servo = Command(4, 'servo', (-90, 90), communicator)
servo = Command(4, 'servo', (left_value, right_value), communicator)

joystick = WiiUJoystick(axis_active_fn=joy_changed)

joystick.start()
communicator.start()

try:
    input("Press enter to exit")
except:
    traceback.print_exc()
finally:
    joystick.stop()
    communicator.stop()
