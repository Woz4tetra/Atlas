"""
Written by Ben Warwick

RoboQuasar3.0, written for the Atlas Project (autonomous buggy group)
Version 3/16/2016
=========

Usage
-----
python __main__.py
- or - (in project's parent directory):
python RoboQuasar3.0

"""
import traceback
import sys
import time
import math

sys.path.insert(0, '../')

from microcontroller.data import Sensor
from microcontroller.data import Command
from microcontroller.data import start, stop, is_running

from analyzers.logger import Recorder
from analyzers.kalman_filter import StateFilter
from analyzers.interpreter import Interpreter
from analyzers.map import Map
from analyzers.binder import Binder
from analyzers.logger import parse

from controllers.gcjoystick import joystick_init
from controllers.servo_map import *

from sound.player import TunePlayer



def find_nearest(map, position):
    map_dist = [0] * len(map)
    for index in range(len(map_dist)):
        dlat = abs(float(map[index][0] - position[0]))
        dlong = abs(float(map[index][1] - position[1]))
        dist = ((dlat ** 2) + (dlong ** 2)) ** 0.5
        map_dist[index] = dist
    smallest_value = min(map_dist)
    index = (map_dist.index(smallest_value) + 2) % len(map)
    return map[index]

def main(log_data=True, manual_mode=True, print_data=False):
    servo_steering = Command(0, 'position', (90, -90))
    # servo_brakes = Command(1, 'position', (90, -90))

    sensor_data = parse("Test Day 4/Sat Mar 12 23;06;53 2016.csv")
    if len(sensor_data[0]) == 23:
        initial_lat = sensor_data[0][2] + sensor_data[0][3] / 60
        initial_lon = sensor_data[0][4] + sensor_data[0][5] / 60
        initial_heading = sensor_data[0][16]
        prev_encoder_value = sensor_data[0][9]
        prev_time = sensor_data[0][0]
    elif len(sensor_data[0]) == 10:
        initial_lat = sensor_data[0][2]
        initial_lon = sensor_data[0][3]
        initial_heading = sensor_data[0][8]
        #currently using yaw rather than heading
        prev_encoder_value = sensor_data[0][5]
        prev_time = sensor_data[0][0]
    else:
        raise Exception("Missing degrees on gps")

    joystick = joystick_init()
    notifier = TunePlayer()

    start(use_handshake=False)

    # 1.344451296765884 for shift_angle?
    interpreter = Interpreter(prev_encoder_value, initial_lat,
                              initial_lon, initial_heading)
    prev_acc = [0,0]
    prev_gps = [0,0]

    kfilter = StateFilter()
    map = Map("From Test Day 4 Long Data Run.csv",
              origin_lat=initial_lat,
              origin_long=initial_lon,
              shift_angle=initial_heading)
    binder = Binder(map)

    for index in range(1, len(sensor_data)):
        row = sensor_data[index]
        if len(row) == 23:
            timestamp, servo, lat_deg, lat_min, lon_deg, lon_min, gps_speed, \
            gps_heading, gps_hdop, encoder_counts, accel_x, accel_y, accel_z, \
            gyro_x, gyro_y, gyro_z, yaw, pitch, roll, quat_w, quat_x, quat_y, \
            quat_z = row
            latitude = lat_deg + lat_min / 60
            longitude = lon_deg + lon_min / 60

        elif len(row) == 10:
            (timestamp, servo, latitude, longitude, gps_heading,
             encoder_counts, accel_x, accel_y, yaw, compass) = row
        else:
            raise ValueError("Invalid file. Not correct data headers")


        #print("%9.8f\t%9.8f\t%9.8f" % (accel_x, accel_y, yaw))

        gps_x, gps_y, enc_counts, yaw = interpreter.convert(
            latitude, longitude, encoder_counts, yaw)

    #cant really deal with figuring out whether encoder was updated or not from
    #here. there isnt enough information that i can find
        enc_flag = False

        if prev_acc == [accel_x, accel_y]:
            acc_flag = True
        else:
            acc_flag = False

        prev_acc = [accel_x, accel_y]

        if prev_gps == [gps_x, gps_y]:
            gps_flag = True
        else:
            gps_flag = False

        x, y, vx, vy, ax, ay = kfilter.update(gps_x, gps_y, enc_counts,
                enc_flag, accel_x, accel_y, yaw,
                timestamp - prev_time, acc_flag, gps_flag)

        prev_time = timestamp

        # print("%9.8f\t%9.8f\t%9.8f\t%9.8f\t%9.8f\t%9.8f\t%9.8f\t%9.8f"
        #         % (gps_x,gps_y,x,y,vx,vy,ax,ay))
        goal_x, goal_y = binder.bind((x, y))# find_nearest(map, (x, y))
        servo_steering["position"] = \
            servo_value((x, y, yaw), (goal_x, goal_y))
        print("%0.4f, (%0.4f, %0.4f), (%0.4f, %0.4f), %i" % (timestamp, x, y, goal_x, goal_y, servo_steering["position"]))

        # time.sleep(0.005)
    notifier.play("PuzzleDone.wav")
    stop()
    time.sleep(1)

if __name__ == '__main__':
    print(__doc__)
    main()
