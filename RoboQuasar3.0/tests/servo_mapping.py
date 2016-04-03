
import traceback
import sys
import time
import math
sys.path.insert(0, '../')

from microcontroller.data import Sensor
from microcontroller.data import Command
from microcontroller.data import start, stop, is_running
from controllers.gcjoystick import joystick_init
from controllers.servo_map import *


servo = Command(0, 'position', (90, -90))

joystick = joystick_init()

start(use_handshake=False)

try:
    while True:
        joystick.update()
        # servo["position"] = int(
        #     50 * -(joystick.triggers.L - joystick.triggers.R))
        # new_servo_value = float(input('servo angle in degrees: '))
        # new_servo_angle = math.pi/180.0 * new_servo_value
        # servo["position"] = map_servo(new_servo_angle)
        print('x,y: ', joystick.mainStick.y, -joystick.mainStick.x)
        servo["position"] = state_to_servo([0, 0, 0], [joystick.mainStick.y, -5.34 / 180 * joystick.mainStick.x])
        print('servo_value: ', state_to_servo([0, 0, 0], [joystick.mainStick.y, -5.34 / 180 * joystick.mainStick.x]))
        time.sleep(0.005)
except:
    traceback.print_exc()
    stop()
