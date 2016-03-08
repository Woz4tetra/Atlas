
import sys

sys.path.insert(0, "../")

from board.logger import parse
from controller.filter import MainFilter

sensor_data = parse("Sat Feb 27 21;46;23 2016-edited.csv")

if len(sensor_data[0]) == 23:
    initial_lat = sensor_data[0][3]
    initial_lon = sensor_data[0][5]
    initial_heading = sensor_data[0][16]
    prev_encoder_value = sensor_data[0][9]
else:
    initial_lat = sensor_data[0][2]
    initial_lon = sensor_data[0][3]
    initial_heading = sensor_data[0][14]
    prev_encoder_value = sensor_data[0][7]


kfilter = MainFilter(initial_lat, initial_lon, 0.1333, initial_heading, prev_encoder_value)

for row in sensor_data[1:]:
    if len(row) == 23:
        timestamp, servo, lat_min, lat_sec, lon_min, lon_sec, gps_speed, \
            gps_heading, gps_hdop, encoder_counts, accel_x, accel_y, accel_z, \
            gyro_x, gyro_y, gyro_z, yaw, pitch, roll, quat_w, quat_x, quat_y, \
            quat_z = row
    else:
        timestamp, servo, lat_min, lat_sec, gps_speed, \
            gps_heading, gps_hdop, encoder_counts, accel_x, accel_y, accel_z, \
            gyro_x, gyro_y, gyro_z, yaw, pitch, roll, quat_w, quat_x, quat_y, \
            quat_z = row
        lon_min, lon_sec = 0, 0.0

    x,y, phi = kfilter.update(lat_sec, lon_sec,
                              encoder_counts, accel_x, accel_y,
                              gyro_z, yaw)
    print ("%9.8f\t%9.8f\t%9.8f" %(x,y,phi) )
