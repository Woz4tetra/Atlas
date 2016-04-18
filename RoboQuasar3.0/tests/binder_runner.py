import sys
import math

sys.path.insert(0, "../")

from analyzers.logger import parse
from analyzers.binder import Binder
from analyzers.converter import Interpreter

sensor_data = parse("Test Day 4/Sat Mar 12 22;38;45 2016.csv")

if len(sensor_data[0]) == 23:
    initial_lat = sensor_data[0][2] + sensor_data[0][3] / 60
    initial_lon = sensor_data[0][4] + sensor_data[0][5] / 60
    initial_heading = sensor_data[0][16]
    prev_encoder_value = sensor_data[0][9]
elif len(sensor_data[0]) == 10:
    initial_lat = sensor_data[0][2]
    initial_lon = sensor_data[0][3]
    initial_heading = sensor_data[0][8]
    #currently using yaw rather than heading
    prev_encoder_value = sensor_data[0][5]
    prev_time = sensor_data[0][0]
else:
    raise Exception("Missing degrees on gps")

interpreter = Interpreter(prev_encoder_value, initial_lat,
                          initial_lon)
binder = Binder("Track Field Map.csv")
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



    gps_x, gps_y, accel_x, accel_y, enc_counts = interpreter.convert(
        latitude, longitude, accel_x, accel_y,
        encoder_counts, yaw)

    bind_x = binder.bind([gps_x, gps_y])

