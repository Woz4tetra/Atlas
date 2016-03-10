import sys

sys.path.insert(0, "../")

from board.logger import parse
from board.filter import StateFilter
from board.interpreter import Interpreter

sensor_data = parse("Sat Feb 27 22;05;19 2016.csv")

if len(sensor_data[0]) == 23:
    initial_lat = sensor_data[0][2] + sensor_data[0][3] / 60
    initial_lon = sensor_data[0][4] + sensor_data[0][5] / 60
    initial_heading = sensor_data[0][16]
    prev_encoder_value = sensor_data[0][9]
else:
    raise Exception("Missing degrees on gps")

interpreter = Interpreter(prev_encoder_value, initial_lat, initial_lon, 1.344451296765884)
kfilter = StateFilter()

for row in sensor_data[1:]:
    timestamp, servo, lat_deg, lat_min, lon_deg, lon_min, gps_speed, \
    gps_heading, gps_hdop, encoder_counts, accel_x, accel_y, accel_z, \
    gyro_x, gyro_y, gyro_z, yaw, pitch, roll, quat_w, quat_x, quat_y, \
    quat_z = row

    latitude = lat_deg + lat_min / 60
    longitude = lon_deg + lon_min / 60

    gps_x, gps_y, enc_counts, gyro_z, yaw = interpreter.convert(
        latitude, longitude, encoder_counts, gyro_z, yaw)

    x, y, phi = kfilter.update(gps_x, gps_y, enc_counts, accel_x, accel_y,
                               gyro_z, yaw, 0.105)
    #print("%9.8f,%9.8f,%9.8f" % (x, y, phi))

    print("%9.8f\t%9.8f\t%9.8f\t%9.8f\t%9.8f\t%9.8f" % (gps_x,x,gps_y,y,enc_counts,phi))
