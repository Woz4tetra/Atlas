import math
import sys

from matplotlib import pyplot as plt

sys.path.insert(0, "../")

from microcontroller.logger import *
from analyzers.buggypi_filter import BuggyPiFilter

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


def gps_bearing(long, lat, prev_long, prev_lat):  # long & lat in radians
    y = math.sin(long - prev_long) * math.cos(lat)
    x = (math.cos(prev_lat) * math.sin(lat) -
         math.sin(prev_lat) * math.cos(lat) *
         math.cos(long - prev_long))
    bearing = math.atan2(y, x)

    return bearing


def plot_everything(file_name, directory=None):
    lat_data, long_data = [], []  # all gps data
    check_lat, check_long = [], []  # checkpoint gps points
    check_lat_data, check_long_data = [], []  # gps points when checkpoint was recorded
    yaw_lines = []
    bearing_lines = []

    checkpoints = get_checkpoints()
    counter = 0

    prev_enc = 0
    prev_enc_time = 0
    prev_time = 0

    prev_long = None
    prev_lat = None

    speed = 0
    x, y = 0, 0
    enc_long, enc_lat = [], []

    yaw = None

    gps_counter = 0

    # initial_long, initial_lat = checkpoints[0]

    for timestamp, name, values in Parser(file_name, directory):
        if name == 'gps':
            if len(lat_data) == 0:
                plt.plot(values['long'], values['lat'], "go", markersize=12)

            lat_data.append(values['lat'])
            long_data.append(values['long'])
            gps_counter += 1

            if prev_long is not None and prev_lat is not None and (
                    long_data[-1] != prev_long or lat_data[-1] != prev_lat):
                bearing = math.atan2(lat_data[-1] - prev_lat,
                                     long_data[-1] - prev_long)
                print("%s\t%s\t%s" % (values['lat'], values['long'], math.degrees(bearing)))

                x1 = long_data[-1] + 0.00001 * math.cos(bearing)
                y1 = lat_data[-1] + 0.00001 * math.sin(bearing)

                bearing_lines.append((long_data[-1], x1))
                bearing_lines.append((lat_data[-1], y1))
                bearing_lines.append('green')

            if long_data[-1] != prev_long:
                prev_long = long_data[-1]
            if lat_data[-1] != prev_lat:
                prev_lat = lat_data[-1]

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
            speed = enc_to_meters(counts - prev_enc) / (
                timestamp - prev_enc_time)
            prev_enc = counts
            prev_enc_time = timestamp

        if yaw is not None:
            x -= speed * math.cos(yaw) * (timestamp - prev_time)
            y -= speed * math.sin(yaw) * (timestamp - prev_time)
            prev_time = timestamp

            dist = (x ** 2 + y ** 2) ** 0.5
            angle = math.atan2(y, x)  # - math.pi - 0.4
            long, lat = dist_to_gps(checkpoints[0][0], checkpoints[0][1], dist,
                                    angle)
            enc_long.append(long)
            enc_lat.append(lat)

        counter += 1

    plt.plot(long_data, lat_data, "r", label="GPS")
    plt.plot(check_long, check_lat, "o", label="Checkpoints")
    plt.plot(check_long_data, check_lat_data, "ro", label="")
    plt.plot(*yaw_lines)
    plt.plot(*bearing_lines)
    plt.plot(long_data[-1], lat_data[-1], "bo", markersize=12, label="Start", )
    plt.plot(enc_long, enc_lat, "g", label="Encoder", )
    plt.legend(loc='upper right', shadow=True, fontsize='x-small')


def plot_filter(file_name, directory=None):
    checkpoints = get_checkpoints()
    initial_long, initial_lat = checkpoints[0]
    pi_filter = BuggyPiFilter(initial_long, initial_lat, 0.0, 6, wheel_radius,
                              0.25)

    prev_filter_time = 0.0

    prev_imu = 0.0
    prev_imu_time = 0.0

    prev_enc = 0
    prev_enc_time = 0.0

    bag = {
        "gps": [initial_long, initial_lat, False],
        "encoder": 0,
        "imu ang v": 0.0,
        "enc v": 0.0,
        "servo": 0
    }

    state_x = []
    state_y = []
    state_heading = []

    heading_counter = 0

    for timestamp, name, values in Parser(file_name, directory):
        # if timestamp - prev_filter_time > 0.01:  # 100 Hz update
        if bag["gps"][2] is False:
            state = pi_filter.update(bag["encoder"], prev_enc_time,
                                     bag["imu ang v"], prev_imu_time,
                                     bag["enc v"], bag["servo"], timestamp)
        else:
            state = pi_filter.update(bag["encoder"], prev_enc_time,
                                     bag["imu ang v"], prev_imu_time,
                                     bag["enc v"], bag["servo"], timestamp,
                                     bag["gps"][0], bag["gps"][1])
            bag["gps"][2] = False
        # prev_filter_time = timestamp
        # print(state)
        state_x.append(state[0])
        state_y.append(state[1])
        if heading_counter % 100 == 0:
            x0 = state_x[-1]
            y0 = state_y[-1]
            x1 = x0 + 0.00001 * math.cos(state[2])
            y1 = y0 + 0.00001 * math.sin(state[2])
            state_heading.append((x0, x1))
            state_heading.append((y0, y1))
            state_heading.append('orange')

        heading_counter += 1

        if name == "gps":
            bag["gps"][0] = values["long"]
            bag["gps"][1] = values["lat"]
            bag["gps"][2] = True
        elif name == "imu":
            bag["imu ang v"] = (values["yaw"] - prev_imu) / (
                timestamp - prev_imu_time)
            prev_imu = values["yaw"]
            prev_imu_time = timestamp
        elif name == "encoder":
            bag["encoder"] = values["counts"]
            bag["enc v"] = pi_filter.enc_to_meters(
                values["counts"] - prev_enc) / (
                               timestamp - prev_enc_time)

            prev_enc = values["counts"]
            prev_enc_time = timestamp
        elif name == "servo":
            bag["servo"] = values[None]

    plt.plot(state_x, state_y, 'r', label="state xy")
    plt.plot(*state_heading)
    plt.legend(loc='upper right', shadow=True, fontsize='x-small')


if __name__ == '__main__':
    if len(sys.argv) == 2:
        file_name = sys.argv[1]
        directory = None
    elif len(sys.argv) == 3:
        file_name = sys.argv[1]
        directory = sys.argv[2]
    else:
        file_name = 0
        directory = "Jun 23 2016"

    try:
        file_name = int(file_name)
    except ValueError:
        pass

    # plot_everything(file_name, directory)
    plot_filter(file_name, directory)
    plt.show()
