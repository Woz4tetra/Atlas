import sys
import math

sys.path.insert(0, "../")

from microcontroller.data import *
from microcontroller.comm import *
from manual.wiiu_joystick import WiiUJoystick


def button_dn(button, params):
    global servo
    if button == "ZR":
        servo.set(servo.get() + 1)
    elif button == "ZL":
        servo.set(servo.get() - 1)
    elif button == "A":


def joy_changed(axis, value, params):
    global servo, joystick
    if axis == "left x":
        servo.set(servo.set(servo.get() + int(value / 10)))
        time.sleep(0.05)


sensor_pool = SensorPool()
communicator = Communicator(sensor_pool, address='/dev/ttyAMA0', log_data=False)
if not communicator.initialized:
    raise Exception("Communicator not initialized...")

servo = Command(4, 'servo', (-90, 90), communicator)

joystick = WiiUJoystick(button_down_fn=button_dn,
                        axis_active_fn=joy_changed)

joystick.start()
communicator.start()

print("Go to leftmost, center, and rightmost positions in that order. "
      "Press A when you've reached each of those points.")
print("ZR and ZL nudge the servo left and right by 1. The left joystick "
      "tells the servo to move at a certain speed.")

try:
    input("Press enter to exit")
except:
    traceback.print_exc()
finally:
    joystick.stop()
    communicator.stop()
