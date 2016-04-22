import sys

import numpy as np
import math
from matplotlib import pyplot as plt

sys.path.insert(0, "../")

from analyzers.logger import Recorder, get_data
from analyzers.converter import Converter
from analyzers.kalman_filter import HeadingFilter, PositionFilter
from analyzers.binder import Binder
from controllers.servo_map import state_to_servo


def test_kalman(log_data=False):
    if log_data:
        log = Recorder(directory="Test Day 6")

    timestamps, data, length = get_data(
        "Test Day 7/Tue Apr 19 22;47;21 2016.csv",
        ["gps long", "gps lat", "gps sleep", "encoder", "imu yaw"],
        density=1)
    gps_long, gps_lat, gps_sleep, encoder, yaw = data

    converter = Converter(gps_long[0], gps_lat[0], 0.000003, 0)  # long, lat
    # 0.000003 is an epsilon

    heading_filter = HeadingFilter()
    position_filter = PositionFilter()

    binder = Binder("Trimmed Minimalist Map.csv", gps_long[0],
                    gps_lat[0])
    bind_x, bind_y = 0, 0

    x, y = [], []
    for index in range(len(binder.map.data)):
        x.append(binder.map.data[index][0])
        y.append(binder.map.data[index][1])
    plt.plot(x, y, 'c')

    x, y = [], []
    gps_xs, gps_ys = [], []
    lines = []

    bind_lines = []

    prev_bind_x, prev_bind_y = 0, 0

    for index in range(1, length):
        if gps_sleep[index] != gps_sleep[index - 1]:
            gps_heading, bind_heading = converter.convert_heading(
                gps_long[index], gps_lat[index], bind_x, bind_y)

            gps_x, gps_y, enc_dist = \
                converter.convert_position(
                    gps_long[index], gps_lat[index], encoder[index])

            heading = heading_filter.update(gps_heading, bind_heading,
                                            -yaw[index])
            kalman_x, kalman_y = position_filter.update(gps_x, gps_y,
                                                        enc_dist,
                                                        gps_sleep[index],
                                                        heading)

            bind_x, bind_y = binder.bind((kalman_x, kalman_y))
            hypotenuse = 10  # 0.5 * ((dx ** 2 + dy ** 2) ** .5)
            # USED FOR DISPLAY

            lines.append((kalman_x, kalman_x + hypotenuse * np.cos(heading)))
            lines.append((kalman_y, kalman_y + hypotenuse * np.sin(heading)))
            lines.append('r')

            if bind_x != prev_bind_x or bind_y != prev_bind_y:
                bind_lines.append((kalman_x, bind_x))
                bind_lines.append((kalman_y, bind_y))
                bind_lines.append('y')

                # if prev_bind_x is not None and prev_bind_y is not None:
                #     bind_heading = np.arctan2(bind_x - prev_bind_y,
                #                               bind_y - prev_bind_x)
                prev_bind_x, prev_bind_y = bind_x, bind_y

            x.append(kalman_x)
            y.append(kalman_y)

            # gps_xs.append(encoder[index] * 0.271 * np.pi * np.cos(heading))
            # gps_ys.append(encoder[index] * 0.271 * np.pi * np.sin(heading))
            gps_xs.append(gps_x)
            gps_ys.append(gps_y)

            relative_goal = math.atan2(bind_y - kalman_y, bind_x - kalman_x) - heading
            # relative_goal = (goal_angle - heading + 2 * math.pi) % (
            #     2 * math.pi) - 2 * math.pi

            servo_pos = state_to_servo([kalman_x, kalman_y, heading],
                                       [bind_x, bind_y])
            if relative_goal > math.pi:
                relative_goal -= 2 * math.pi
            if relative_goal < -math.pi:
                relative_goal += 2 * math.pi
            print(kalman_x, kalman_y, relative_goal, heading, servo_pos, "left" if servo_pos - heading < -23 else "right")

    plt.plot(*lines)
    plt.plot(*bind_lines)
    plt.plot(x, y, 'b')
    plt.plot(gps_xs, gps_ys, 'g')

    axes = plt.gca()
    axes.set_xlim([-140, 70])
    axes.set_ylim([-130, 80])

    plt.show()


def test_binder():
    timestamps, data, length = get_data(
        "Test Day 9/Thu Apr 21 23;04;06 2016.csv",
        ["kalman x", "kalman y", "kalman heading", "gps sleep", "gps long",
         "gps lat"],
        density=1)
    kalman_x, kalman_y, kalman_heading, gps_sleep, gps_long, gps_lat = data

    binder = Binder("Trimmed Tue Apr 19 22;47;21 2016 GPS Map.csv", gps_long[0],
                    gps_lat[0])

    #first plot the map
    x, y = [], []
    for index in range(len(binder.map.data)):
        x.append(binder.map.data[index][0])
        y.append(binder.map.data[index][1])
    plt.plot(x, y, 'b')

    prev_bound_x, prev_bound_y = None, None
    bound_x, bound_y = 0, 0
    bind_heading = 0

    lines1 = []
    lines2 = []
    hypotenuse = 10
    value = 50
    for index in range(1, length):
        if gps_sleep[index] != gps_sleep[index - 1]:
            lines1.append((kalman_x[index],
                           kalman_x[index] + hypotenuse * np.cos(
                               kalman_heading[index])))
            lines1.append((kalman_y[index],
                           kalman_y[index] + hypotenuse * np.sin(
                               kalman_heading[index])))
            lines1.append('r')
#            print(kalman_x[index], kalman_y[index])

            (goal_x, goal_y), (bound_x, bound_y) = binder.bind(
                    (kalman_x[index], kalman_y[index]))
            print ("dx: %f\tdy: %f" %(bound_x-kalman_x[index],
                                      bound_y-kalman_y[index]))
           # if bound_x != prev_bound_x or bound_y != prev_bound_y:
            lines2.append((kalman_x[index], bound_x))
            lines2.append((kalman_y[index], bound_y))
            value += 10
            if value >= 255:
                value = 0
            lines2.append("#%0.2x%0.2x%0.2x" % (0, value, value // 2))

            # if prev_bound_x is not None and prev_bound_y is not None:
            #     bind_heading = np.arctan2(bound_y - prev_bound_y,
            #                               bound_x - prev_bound_x)
            prev_bound_x, prev_bound_y = bound_x, bound_y

    # pprint(lines2)
    plt.plot(*lines1)
    plt.plot(*lines2)
    plt.plot(kalman_x, kalman_y, 'r')

    plt.show()


if __name__ == '__main__':
    print(__doc__)
    test_binder()
    #test_kalman()
