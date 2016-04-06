"""
Written by Ben Warwick

diagnose_system.py, written for RoboQuasar3.0
Version 3/10/2015
=========

Checks all incoming data from the robot to make sure everything is operating
as expected
"""
import sys
import time

sys.path.insert(0, "../")

from microcontroller.data import Sensor
from microcontroller.data import Command
from microcontroller.data import start, stop, is_running

from controllers.joystick import joystick_init

gps = Sensor(1, ['lat','long', 'heading'])
encoder = Sensor(2, 'counts')
imu = Sensor(3, ['accel_x', 'accel_y', 'gyro_z', 'yaw', 'compass'])

#controls
servo_steering = Command(0, 'position', (90,-90))

joystick = joystick_init()

start(use_handshake = False)

#wait 5 seconds for the sensors to initialize and start giving data
time.sleep(5)

if (is_running()):
    print("The serial connection is running")
else:
    raise Exception ("The serial connection fails")


#now start diagnosing things
#first, diagnose the imu

accel_x, accel_y, gyro_z =imu['accel_x'],imu['accel_y'],imu['gyro_z']

if (accel_x != 0.0 and accel_y != 0.0 and gyro_z != 0.0):
    print ("imu is giving non-zero data")
else:
    print ("imu is not giving non-zero data")

#now move on to the gps

latitude, longitude = gps['lat'], gps['long']
#NOTE diagnosis is tailored to this geographic location
if (int(latitude) == 40 and int(longitude) == 79):
    print ("gps is giving data within range of location")
else:
    print("gps is not giving fata within range of location")


#now figure out if the servo is reading and taking correct commands

servo_steering['position'] = 90
time.sleep(2)
print("If the wheel is pointed towards the right, then it is working")
print("Otherwise, the servo is not taking commands")
servo_steering['position'] = -23
time.sleep(2)

print("The wheel should now be pointed straight")



#now diagnose the encoder

print("starting to diagnose the encoder")
print("move the buggy at least one wheel rotation in the next 5 seconds")

initial_encoder = encoder['counts']

time.sleep(5)

second_encoder = encoder['counts']

num_rot = second_encoder - initial_encoder

if num_rot > 0:
    print("Successfully completed at least one rotation")
else:
    print("Did not successfully complete one rotation")
