import math
import sys

from matplotlib import pyplot as plt

sys.path.insert(0, "../")

from microcontroller.logger import *


def plot_gps(file_name, directory=None):
    lat, long = [], []
    parser = Parser(file_name, directory)
    for data in parser:
        if data[1] == 'gps':
            lat.append(data[2]['lat'])
            long.append(data[2]['long'])
    plt.plot(long, lat, 'g')


def plot_gps_checkpoints(file_name, directory=None):
    lat_data, long_data = [], []  # all gps data
    check_lat, check_long = [], []  # checkpoint gps points
    check_lat_data, check_long_data = [], []  # gps points when checkpoint was recorded

    parser = Parser(file_name, directory)
    checkpoints = get_points()

    for data in parser:
        if data[1] == 'gps':
            lat_data.append(data[2]['lat'])
            long_data.append(data[2]['long'])
        elif data[1] == 'checkpoint':
            lat, long = checkpoints[data[2]['num']]
            check_lat.append(lat)
            check_long.append(long)

            lat, long = data[2]['lat'], data[2]['long']
            check_lat_data.append(lat)
            check_long_data.append(long)
    plt.plot(long_data, lat_data, "r")
    plt.plot(check_long, check_lat, "o")
    plt.plot(check_long_data, check_lat_data, "ro")


def plot_all_gps(directory):
    lat, long = [], []
    for file_name in os.listdir(directories.get_dir(":logs") + directory):
        parser = Parser(file_name, directory)
        for data in parser:
            if data[1] == 'gps':
                if data[2]['lat'] > 0 and data[2]['long'] > 0:
                    lat.append(data[2]['lat'])
                    long.append(data[2]['long'])
        plt.plot(long, lat)
        lat, long = [], []


def plot_vs_time(file_name, name, value=None, directory=None):
    plot_data = []
    t = []
    parser = Parser(file_name, directory)
    for data in parser:
        if data[1] == name:
            plot_data.append(data[2][value])
            t.append(data[0])
    plt.plot(t, plot_data)


def plot_everything(file_name, directory=None):
    lat_data, long_data = [], []  # all gps data
    check_lat, check_long = [], []  # checkpoint gps points
    check_lat_data, check_long_data = [], []  # gps points when checkpoint was recorded
    yaw_lines = []

    checkpoints = get_points()
    counter = 0
    for data in Parser(file_name, directory):
        if data[1] == 'gps':
            if len(lat_data) == 0:
                plt.plot(data[2]['long'], data[2]['lat'], "go", markersize=12)

            lat_data.append(data[2]['lat'])
            long_data.append(data[2]['long'])

        elif data[1] == 'checkpoint':
            lat, long = checkpoints[data[2]['num']]
            check_lat.append(lat)
            check_long.append(long)

            lat, long = data[2]['lat'], data[2]['long']
            check_lat_data.append(lat)
            check_long_data.append(long)
        elif data[1] == 'imu' and counter % 100 == 0:
            yaw = -data[2]['yaw']

            if len(lat_data) > 0 and len(long_data) > 0:
                x1 = long_data[-1] + 0.00001 * math.cos(yaw)
                y1 = lat_data[-1] + 0.00001 * math.sin(yaw)
                yaw_lines.append((long_data[-1], x1))
                yaw_lines.append((lat_data[-1], y1))
                yaw_lines.append('orange')
        counter += 1

    plt.plot(long_data, lat_data, "r")
    plt.plot(check_long, check_lat, "o")
    plt.plot(check_long_data, check_lat_data, "ro")
    plt.plot(*yaw_lines)
    plt.plot(long_data[-1], lat_data[-1], "bo", markersize=12)


# retrieved from http://www.geomidpoint.com/destination/
# needs to be very exact or else wild errors can occur
earth_radius = 6372797.6


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


def plot_encoder_xy(file_name, directory=None, use_meters=True):
    x, y = 0.0, 0.0
    speed = 0.0
    # yaw = 0.0

    parser = Parser(file_name, directory)

    gps_data = parser.get_first('gps')
    initial_long, initial_lat = gps_data[2]['long'], gps_data[2]['lat']
    if use_meters:
        plt.plot(x, y, "o")
    else:
        plt.plot(initial_long, initial_lat, "o")

    xs, ys = [], []

    prev_enc_time = 0
    prev_time = 0
    yaw = 0.0

    for data in parser:
        timestamp, name, values = data
        if name == 'encoder':
            speed = 0.085 / 6 * math.pi / (timestamp - prev_enc_time)
            prev_enc_time = timestamp
        elif name == 'imu':
            yaw = -values['yaw']  # - math.pi / 2

        x -= speed * math.cos(yaw) * (timestamp - prev_time)
        y -= speed * math.sin(yaw) * (timestamp - prev_time)
        prev_time = timestamp

        if use_meters:
            xs.append(x)
            ys.append(y)
        else:
            dist = (x ** 2 + y ** 2) ** 0.5
            angle = math.atan2(y, x)  # - math.pi - 0.4
            long, lat = dist_to_gps(initial_long, initial_lat, dist, angle)
            xs.append(long)
            ys.append(lat)

    plt.plot(xs, ys, "r")


if __name__ == '__main__':
    file_name = 3

    # plot_vs_time(file_name, 'imu', 'yaw', directory="Jun 22 2016")
    # plot_vs_time(file_name, 'encoder', 'counts', directory="Jun 22 2016")
    plot_encoder_xy(file_name, "Jun 22 2016", use_meters=False)
    plot_everything(file_name, "Jun 22 2016")
    plt.show()

    # parser = Parser("Wed Jun 22 20;59;40 2016")
    #
    # for data in parser:
    #    if data[1] == 'imu':
    #        print(data)
