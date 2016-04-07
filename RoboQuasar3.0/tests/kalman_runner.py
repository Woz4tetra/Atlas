import sys
import math

sys.path.insert(0, "../")

from analyzers.logger import parse
from analyzers.kalman_filter import PositionFilter
from analyzers.converter import Interpreter

sensor_data = parse("Test Day 4/Sat Mar 12 23;06;53 2016.csv")

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
                          initial_lon, initial_heading)
# this value was 1.344451296765884 (added 2 to it)
# then made it intial_heading * pi/180



kfilter = PositionFilter()

prev_acc = [0,0]
prev_gps = [0,0]

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


    #print("%9.8f\t%9.8f\t%9.8f" % (accel_x, accel_y, yaw))

    gps_x, gps_y, enc_counts, yaw = interpreter.convert(
        latitude, longitude, encoder_counts, yaw)

#cant really deal with figuring out whether encoder was updated or not from
#here. there isnt enough information that i can find
    enc_flag = False
    gps_flag = False

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

    print("%9.8f\t%9.8f\t%9.8f\t%9.8f\t%9.8f\t%9.8f\t%9.8f\t%9.8f"
            % (gps_x,gps_y,x,y,vx,vy,ax,ay))
