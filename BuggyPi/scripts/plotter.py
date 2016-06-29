import math
import sys

from matplotlib import pyplot as plt

sys.path.insert(0, "../")

from analyzers.kalman_filter import BuggyPiFilter
from microcontroller.logger import *

# retrieved from http://www.geomidpoint.com/destination/
# needs to be very exact or else wild errors can occur
earth_radius = 6372797.6
wheel_radius = 0.097

def dist_to_gps(initial_long, initial_lat, distance, bearing):
    initial_long = math.radians(initial_long)
    initial_lat = math.radians(initial_lat)

    lat = math.asin(math.sin(initial_lat) * math.cos(distance / earth_radius) +
                    math.cos(initial_lat) * math.sin(distance / earth_radius) *
                    math.cos(bearing))
    long = (initial_long +
            math.atan2(math.sin(bearing) * math.sin(distance / earth_radius) *
                       math.cos(initial_lat),
                       math.cos(distance / earth_radius) - math.sin(initial_lat)
                       * math.sin(lat)))
    return math.degrees(long), math.degrees(lat)


def enc_to_meters(counts):
    return counts * wheel_radius / 6 * math.pi


def plot_everything(file_name, directory=None):
    lat_data, long_data = [], []  # all gps data
    check_lat, check_long = [], []  # checkpoint gps points
    check_lat_data, check_long_data = [], []  # gps points when checkpoint was recorded
    yaw_lines = []

    checkpoints = get_points()
    counter = 0

    prev_enc = 0
    prev_enc_time = 0
    prev_time = 0

    speed = 0
    x, y = 0, 0
    enc_long, enc_lat = [], []

    yaw = None

    # initial_long, initial_lat = checkpoints[0]

    for timestamp, name, values in Parser(file_name, directory):
        if name == 'gps':
            if len(lat_data) == 0:
                plt.plot(values['long'], values['lat'], "go", markersize=12)

            lat_data.append(values['lat'])
            long_data.append(values['long'])

        elif name == 'checkpoint':
            long, lat = checkpoints[values['num']]
            check_long.append(long)
            check_lat.append(lat)

            long, lat = values['long'], values['lat']
            check_long_data.append(long)
            check_lat_data.append(lat)
        elif name == 'imu':
            yaw = values['yaw']

            if counter % 100 == 0 and len(lat_data) > 0 and len(long_data) > 0:
                x1 = long_data[-1] + 0.00001 * math.cos(-yaw)
                y1 = lat_data[-1] + 0.00001 * math.sin(-yaw)
                yaw_lines.append((long_data[-1], x1))
                yaw_lines.append((lat_data[-1], y1))
                yaw_lines.append('orange')

        elif name == 'encoder':
            counts = values['counts']
            speed = enc_to_meters(counts - prev_enc) / (timestamp - prev_enc_time)
            prev_enc = counts
            prev_enc_time = timestamp

        if yaw is not None:
            x -= speed * math.cos(yaw) * (timestamp - prev_time)
            y -= speed * math.sin(yaw) * (timestamp - prev_time)
            prev_time = timestamp

            dist = (x ** 2 + y ** 2) ** 0.5
            angle = math.atan2(y, x)  # - math.pi - 0.4
            long, lat = dist_to_gps(checkpoints[0][0], checkpoints[0][1], dist, angle)
            enc_long.append(long)
            enc_lat.append(lat)

        counter += 1

    plt.plot(long_data, lat_data, "r", label="GPS")
    plt.plot(check_long, check_lat, "o", label="Checkpoints")
    plt.plot(check_long_data, check_lat_data, "ro", label="")
    plt.plot(*yaw_lines)
    plt.plot(long_data[-1], lat_data[-1], "bo", markersize=12, label="Start",)
    plt.plot(enc_long, enc_lat, "g", label="Encoder",)
    plt.legend(loc='upper right', shadow=True, fontsize='x-small')


if __name__ == '__main__':
    if len(sys.argv) == 2:
        file_name = sys.argv[1]
        directory = None
    elif len(sys.argv) == 3:
        file_name = sys.argv[1]
        directory = sys.argv[2]
    else:
        file_name = 7
        directory = "Jun 23 2016"

    try:
        file_name = int(file_name)
    except ValueError:
        pass

    plot_everything(file_name, directory)
    plt.show()
