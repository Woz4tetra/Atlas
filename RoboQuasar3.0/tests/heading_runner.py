import sys
import math

sys.path.insert(0, "../")

from analyzers.logger import parse
from analyzers.kalman_filter import HeadingFilter
from analyzers.interpreter import Interpreter

sensor_data = parse("Test Day 4/Sat Mar 12 23;06;53 2016.csv")

kfilter = HeadingFilter()

prev_compass = sensor_data[0][9]
prev_gps_heading = sensor_data[0][4]

for row in sensor_data[1:]:

    if len(row) == 23 :
        timestamp, servo, lat_deg, lat_min, lon_deg, lon_min, gps_speed, \
        gps_heading, gps_hdop, encoder_counts, accel_x, accel_y, accel_z, \
        gyro_x, gyro_y, gyro_z, yaw, pitch, roll, quat_w, quat_x, quat_y, \
        quat_z = row
        latitude = lat_deg + lat_min / 60
        longitude = lon_deg + lon_min / 60

    elif len(row == 10):
        (timestamp, servo, latitude, longitude, gps_heading,
         encoder_counts, accel_x, accel_y, yaw, compass) = row

    imu_flag = False
    gps_flag = False

    if prev_compass == compass:
        imu_flag = True
    if prev_gps_heading == gps_heading:
        gps_flag = True

    prev_compass = compass
    prev_gps_heading = gps_heading

    [filtered_heading] = \
        kfilter.update(compass, compass, imu_flag, imu_flag)

    print ("%9.8f\t%9.8f" %(filtered_heading, compass))
