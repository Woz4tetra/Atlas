import sys
import time
import numpy as np
from matplotlib import pyplot as plt

sys.path.insert(0, "../")

from analyzers.logger import Recorder, get_data
from analyzers.converter import Converter
from analyzers.kalman_filter import HeadingFilter, PositionFilter
from analyzers.binder import Binder


def test_kalman(log_data=False):
    if log_data:
        log = Recorder(directory="Test Day 6")

    timestamps, data, length = get_data(
        "Test Day 5/Sun Apr 10 18;30;08 2016.csv",
        ["gps_long", "gps_lat", "gps_sleep", "gps_flag", "encoder", "yaw"],
        density=1)
    gps_long, gps_lat, gps_sleep, gps_flag, encoder, yaw = data

    converter = Converter(gps_long[0], gps_lat[0], 0.000003, 0)  # long, lat
    #0.000003 is an epsilon

    heading_filter = HeadingFilter()
    position_filter = PositionFilter()

    binder = Binder("Track Field Map.csv", gps_long[0], gps_lat[0])
    bind_x, bind_y = 0, 0


    x, y = [], []
    for index in range(len(binder.map.data)):
        x.append(binder.map.data[index][0])
        y.append(binder.map.data[index][1])
    plt.plot(x, y)

    x, y = [], []
    gps_xs, gps_ys = [], []
    lines = []

    new_x, new_y = 0,0
    prev_gps_dt = None
    for index in range(1, length):
        upd_dt = timestamps[index] - timestamps[index - 1]
        gps_dt = gps_sleep[index]
        #print(gps_heading, heading, bind_heading)
        if gps_dt != prev_gps_dt:
            gps_heading, bind_heading = converter.convert_heading(
                gps_long[index], gps_lat[index], bind_x, bind_y)
            gps_x, gps_y, enc_dist = \
                converter.convert_position(
                    gps_long[index], gps_lat[index], encoder[index])
            heading = heading_filter.update(gps_heading, bind_heading, yaw[index])
            new_x, new_y = position_filter.update(gps_x, gps_y,
                                        enc_dist,gps_dt, heading)
            print(new_x,new_y)

            prev_gps_dt = gps_dt

            bind_x, bind_y = binder.bind((new_x, new_y))
            hypotenuse = 10  # 0.5 * ((dx ** 2 + dy ** 2) ** .5)
            #USED FOR DISPLAY

            lines.append((new_x, new_x + hypotenuse * np.cos(heading)))
            lines.append((new_y, new_y + hypotenuse * np.sin(heading)))
            lines.append('r')

            x.append(new_x)
            y.append(new_y)
        # gps_xs.append(gps_x)
        # gps_ys.append(gps_y)

    plt.plot(*lines)
    plt.plot(x, y)
    # plt.plot(gps_xs, gps_ys, 'p')

    plt.show()

def test_binder():
    timestamps, data, length = get_data(
        "Test Day 7/Tue Apr 19 22;58;26 2016.csv",
        ["kalman x", "kalman y", "kalman heading", "gps sleep", "gps long", "gps lat"],
        density=1)
    kalman_x, kalman_y, kalman_heading, gps_sleep, gps_long, gps_lat = data

    binder = Binder("Trimmed Tue Apr 19 22;47;21 2016 GPS Map.csv", gps_long[0], gps_lat[0])
    x, y = [], []
    for index in range(len(binder.map.data)):
        x.append(binder.map.data[index][0])
        y.append(binder.map.data[index][1])
    plt.plot(x, y)

    prev_bound_x, prev_bound_y = None, None

    lines1 = []
    lines2 = []
    hypotenuse = 10
    value = 50
    for index in range(1, length):
        if gps_sleep[index] != gps_sleep[index - 1]:
            lines1.append((kalman_x[index], kalman_x[index] + hypotenuse * np.cos(kalman_heading[index])))
            lines1.append((kalman_y[index], kalman_y[index] + hypotenuse * np.sin(kalman_heading[index])))
            lines1.append('r')
            print(kalman_x[index], kalman_y[index])

            bound_x, bound_y = binder.bind((kalman_x[index], kalman_y[index]))
            if bound_x != prev_bound_x or bound_y != prev_bound_y:
                lines2.append((kalman_x[index], bound_x))
                lines2.append((kalman_y[index], bound_y))
                value += 10
                if value >= 255:
                    value = 0
                lines2.append("#%0.2x%0.2x%0.2x" % (0, value, value))
                prev_bound_x, prev_bound_y = bound_x, bound_y

    from pprint import pprint
    # pprint(lines2)
    plt.plot(*lines1)
    plt.plot(*lines2)
    plt.plot(kalman_x, kalman_y)

    plt.show()

if __name__ == '__main__':
    print(__doc__)
    test_binder()
