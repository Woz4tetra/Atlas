"""
Written by Ben Warwick

RoboQuasar3.0, written for the Atlas Project (autonomous buggy group)
Version 4/22/2016
=========

A test file meant to make data analysis visual and easy. Hopefully this file
will allow for insights into what's wrong with the system.

"""

import os
import sys
import traceback

import numpy as np
from matplotlib import pyplot as plt

sys.path.insert(0, "../")

import config
from analyzers.kalman_filter import HeadingFilter, PositionFilter
from analyzers.converter import Converter
from analyzers.binder import Binder
from analyzers.map import Map
from analyzers.logger import get_data

from microcontroller.data import *
from microcontroller.dashboard import *
from controllers.servo_map import *


def populate_lines(kalman_x, kalman_y, kalman_heading, binder, axes,
                   heading_lines, bind_lines, goal_lines,
                   plot_heading, plot_binds, plot_goals, plot_kalman,
                   line_length=10,
                   bind_x0=None, bind_y0=None,
                   goal_x0=None, goal_y0=None):

    if plot_binds or plot_goals or plot_heading or plot_kalman:
        bind_x, bind_y, goal_x, goal_y = \
            binder.bind((kalman_x, kalman_y))
        if bind_x0 is not None:
            bind_x = bind_x0
        if bind_y0 is not None:
            bind_y = bind_y0
        if goal_x0 is not None:
            goal_x = goal_x0
        if goal_y0 is not None:
            goal_y = goal_y0

        if plot_binds:
            bind_lines.append((kalman_x, bind_x))
            bind_lines.append((kalman_y, bind_y))
            bind_lines.append('brown')

            axes.arrow(bind_x, bind_y,
                       goal_x - bind_x, goal_y - bind_y,
                       color='midnightblue',
                       head_width=3)

        if plot_goals:
            goal_lines.append((kalman_x, goal_x))
            goal_lines.append((kalman_y, goal_y))
            goal_lines.append('goldenrod')

        if plot_heading:
            if plot_kalman:
                heading_lines.append((kalman_x,
                                      kalman_x + line_length * math.cos(
                                          kalman_heading)))
                heading_lines.append((kalman_y,
                                      kalman_y + line_length * math.sin(
                                          kalman_heading)))
                heading_lines.append('r')
            elif not plot_kalman and (plot_binds or plot_goals):
                heading_lines.append((bind_x,
                                      bind_x + line_length * math.cos(
                                          kalman_heading)))
                heading_lines.append((bind_y,
                                      bind_y + line_length * math.sin(
                                          kalman_heading)))
                heading_lines.append('r')
        print(state_to_servo(
                [bind_x, bind_y, kalman_heading],
                [goal_x, goal_y]),
            state_to_angle(
                [bind_x, bind_y, kalman_heading],
                [goal_x, goal_y]
            ))

        return bind_x, bind_y, goal_x, bind_y

    else:
        return 0, 0, 0, 0


reference_map = Map("Map from Wed Apr 20 21;51;46 2016.csv")


def test_with_servo(data_set, map_name,
                    plot_map=True, plot_kalman=True,
                    plot_heading=True, plot_binds=True, plot_goals=True,
                    initial_gps=None, enable_servo=True):
    if data_set == 'random':
        data_sets = []
        for root, dirs, files in os.walk(config.get_dir(":logs"),
                                         topdown=False):
            for name in files:
                if name.endswith(".csv"):
                    data_sets.append(root + "/" + name)

        data_set = random.choice(data_sets)
        print("picked:", data_set)
        log_index = data_set.find("logs")
        data_set = data_set[log_index + len("logs") + 1:]

    timestamps, data, length = get_data(
        data_set, ["gps long", "gps lat", "gps sleep", "encoder", "imu yaw"],
        density=1)

    gps_long, gps_lat, gps_sleep, encoder, yaw = data

    heading_filter = HeadingFilter()
    position_filter = PositionFilter()

    if initial_gps is None:
        initial_gps = gps_long[0], gps_lat[0]
    elif type(initial_gps) == int:
        initial_gps = reference_map[initial_gps]

    converter = Converter(initial_gps[0], initial_gps[1], 0.000003, 0)
    binder = Binder(map_name, initial_gps[0], initial_gps[1])

    bind_x, bind_y = 0, 0

    if enable_servo:
        servo_steering = Command(0, 'position', (-90, 90))

        start()
    else:
        servo_steering = None

    plt.ion()

    if plot_map:
        plt.plot(binder.map[:, 0], binder.map[:, 1], 'c')
    kalman_graph = plt.plot(np.zeros_like(binder.map))[0]
    axes = plt.axes()

    ymin = float(min(binder.map[:, 1])) - 10
    ymax = float(max(binder.map[:, 1])) + 10
    plt.ylim([ymin, ymax])

    xmin = float(min(binder.map[:, 0])) - 10
    xmax = float(max(binder.map[:, 0])) + 10
    plt.xlim([xmin, xmax])

    arrow_color = 0

    line_length = 10

    kalman_data = [[], []]

    try:
        if plot_heading or plot_binds or plot_goals or plot_kalman:
            for index in range(1, length):
                if gps_sleep[index] != gps_sleep[index - 1]:
                    gps_heading, bind_heading = converter.convert_heading(
                        gps_long[index], gps_lat[index], bind_x, bind_y)

                    gps_x0, gps_y0, enc_dist = \
                        converter.convert_position(
                            gps_long[index], gps_lat[index], encoder[index])

                    kalman_heading = heading_filter.update(
                        gps_heading, bind_heading, yaw[index],
                        gps_sleep[index])
                    kalman_x0, kalman_y0 = position_filter.update(
                        gps_x0, gps_y0, enc_dist, gps_sleep[index],
                        kalman_heading)

                    prev_bind_x = bind_x
                    prev_bind_y = bind_y
                    bind_x, bind_y, goal_x, goal_y = \
                        binder.bind((kalman_x0, kalman_y0))

                    if plot_binds:
                        if bind_x == prev_bind_x and bind_y == prev_bind_y:
                            arrow_color += 40
                            if arrow_color > 255:
                                arrow_color = 0
                        else:
                            arrow_color = 0

                        axes.arrow(bind_x, bind_y,
                                   goal_x - bind_x, goal_y - bind_y,
                                   color="#%0.2x%0.2x%0.2x" % (
                                       0, arrow_color, arrow_color
                                   ),
                                   head_width=2)
                        plt.plot([kalman_x0, bind_x], [kalman_y0, bind_y],
                                 'brown')

                    if plot_kalman:
                        kalman_data[0].append(kalman_x0)
                        kalman_data[1].append(kalman_y0)

                        kalman_graph.set_data(kalman_data)

                    if plot_heading:
                        if plot_kalman:
                            plt.plot([kalman_x0,
                                      kalman_x0 + line_length * math.cos(
                                          kalman_heading)],
                                     [kalman_y0,
                                      kalman_y0 + line_length * math.sin(
                                          kalman_heading)],
                                     'r')
                        elif plot_binds:
                            plt.plot([bind_x,
                                      bind_x + line_length * math.cos(
                                          kalman_heading)],
                                     [bind_y,
                                      bind_y + line_length * math.sin(
                                          kalman_heading)],
                                     'r')
                    if plot_kalman:
                        if kalman_x0 < xmin:
                            xmin = kalman_x0 - 10
                            plt.xlim([xmin, xmax])
                        if kalman_x0 > xmax:
                            xmax = kalman_x0 + 10
                            plt.xlim([xmin, xmax])

                        if kalman_y0 < ymin:
                            ymin = kalman_y0 - 10
                            plt.ylim([ymin, ymax])
                        if kalman_y0 > ymax:
                            ymax = kalman_y0 + 10
                            plt.ylim([ymin, ymax])

                    plt.draw()
                    plt.pause(0.001)

                    if enable_servo:
                        # input()
                        servo_steering["position"] = state_to_servo(
                            [bind_x, bind_y, kalman_heading],
                            [goal_x, goal_y]
                        )
                        relative_goal = math.atan2(goal_y - bind_y, goal_x - bind_x) - kalman_heading

                        if relative_goal > math.pi:
                            relative_goal -= 2 * math.pi
                        if relative_goal < -math.pi:
                            relative_goal += 2 * math.pi

                        print(servo_steering["position"],
                              "right" if servo_steering[
                                             "position"] < -23 else "left",
                              kalman_heading,
                              relative_goal)

    except KeyboardInterrupt:
        traceback.print_exc()

    finally:
        plt.ioff()
        if enable_servo:
            reset()
            stop()


def test_system(data_set, map_name, plot_type, x_lim=None, y_lim=None,
                plot_map=True, plot_gps=True, plot_kalman=True,
                plot_heading=True, plot_binds=True, plot_goals=True,
                initial_gps=None):
    if plot_type == "kalman":  # recalculate kalman filter, binder, and goal
        sensors = ["gps long", "gps lat", "gps sleep", "encoder", "imu yaw"]
    elif plot_type == "goal":  # recalculate goal only
        sensors = ["gps long", "gps lat", "gps sleep", "gps x", "gps y",
                   "kalman x", "kalman y", "kalman heading",
                   "bind x", "bind y"]
    elif plot_type == "display":  # recalculate nothing
        sensors = ["gps long", "gps lat", "gps sleep", "gps x", "gps y",
                   "kalman x", "kalman y", "kalman heading",
                   "bind x", "bind y", "goal x", "goal y"]
    elif plot_type == "binder":  # recalculate binder and goal
        sensors = ["gps long", "gps lat", "gps sleep", "gps x", "gps y",
                   "kalman x", "kalman y", "kalman heading"]
    elif plot_type == "map":  # just draw the map
        sensors = ["gps long", "gps lat"]
    else:
        raise ValueError("Please provide valid plot type: ", plot_type)

    if data_set == 'random':
        data_sets = []
        for root, dirs, files in os.walk(config.get_dir(":logs"),
                                         topdown=False):
            for name in files:
                if name.endswith(".csv"):
                    data_sets.append(root + "/" + name)

        data_set = random.choice(data_sets)
        print("picked:", data_set)
        log_index = data_set.find("logs")
        data_set = data_set[log_index + len("logs") + 1:]

    timestamps, data, length = get_data(
        data_set, sensors, density=1)

    # if (initial_gps is None or ((initial_gps[0] - data[0][0]) > 0.000003 and (
    #         initial_gps[1] - data[0][1]) > 0.000003)):
    #     initial_gps = data[0][0], data[1][0]
    if initial_gps is None:
        initial_gps = data[0][0], data[1][0]
    elif type(initial_gps) == int:
        initial_gps = reference_map[initial_gps]

    binder = Binder(map_name, initial_gps[0], initial_gps[1])

    if plot_map:
        plt.plot(binder.map[:, 0], binder.map[:, 1], 'c')

    plt.title(plot_type + ": " + data_set)
    axes = plt.gca()

    heading_lines = []
    bind_lines = []
    goal_lines = []

    if plot_type == "kalman":
        gps_long, gps_lat, gps_sleep, encoder, yaw = data

        heading_filter = HeadingFilter()
        position_filter = PositionFilter()

        converter = Converter(initial_gps[0], initial_gps[1], 0.000003, 0)

        bind_x, bind_y = 0, 0
        gps_x, gps_y = [], []
        kalman_x, kalman_y = [], []

        if plot_heading or plot_binds or plot_goals or plot_kalman:
            for index in range(1, length):
                if gps_sleep[index] != gps_sleep[index - 1]:
                    gps_heading, bind_heading = converter.convert_heading(
                        gps_long[index], gps_lat[index], bind_x, bind_y)

                    gps_x0, gps_y0, enc_dist = \
                        converter.convert_position(
                            gps_long[index], gps_lat[index], encoder[index])

                    kalman_heading = heading_filter.update(
                        gps_heading, bind_heading, -yaw[index],
                        gps_sleep[index])
                    kalman_x0, kalman_y0 = position_filter.update(
                        gps_x0, gps_y0, enc_dist, gps_sleep[index],
                        kalman_heading)

                    gps_x.append(gps_x0)
                    gps_y.append(gps_y0)

                    kalman_x.append(kalman_x0)
                    kalman_y.append(kalman_y0)

                    bind_x, bind_y, goal_x, goal_y = populate_lines(
                        kalman_x0, kalman_y0, kalman_heading, binder, axes,
                        heading_lines, bind_lines, goal_lines, plot_heading,
                        plot_binds, plot_goals, plot_kalman)

    elif plot_type == "display":
        gps_long, gps_lat, gps_sleep, gps_x, gps_y, kalman_x, kalman_y, kalman_heading, bind_x, bind_y, goal_x, goal_y = data
        axes = plt.gca()

        if plot_heading or plot_binds or plot_goals or plot_kalman:
            for index in range(1, length):
                if gps_sleep[index] != gps_sleep[index - 1]:
                    populate_lines(kalman_x[index], kalman_y[index],
                                   kalman_heading[index], binder,
                                   axes, heading_lines, bind_lines, goal_lines,
                                   plot_heading, plot_binds, plot_goals,
                                   plot_kalman,
                                   bind_x0=bind_x[index], bind_y0=bind_y[index],
                                   goal_x0=goal_x[index], goal_y0=goal_y[index])
                    print("servo:", state_to_servo(
                        [bind_x[index], bind_y[index], kalman_heading[index]],
                        [goal_x[index], goal_y[index]]))

    elif plot_type == "binder":
        gps_long, gps_lat, gps_sleep, gps_x, gps_y, kalman_x, kalman_y, kalman_heading = data

        if plot_heading or plot_binds or plot_goals or plot_kalman:
            for index in range(1, length):
                if gps_sleep[index] != gps_sleep[index - 1]:
                    populate_lines(kalman_x[index], kalman_y[index],
                                   kalman_heading[index], binder,
                                   axes, heading_lines, bind_lines, goal_lines,
                                   plot_heading, plot_binds, plot_goals,
                                   plot_kalman)

    if x_lim is not None:
        axes.set_xlim([x_lim[0], x_lim[1]])
    if y_lim is not None:
        axes.set_ylim([y_lim[0], y_lim[1]])

    if plot_goals:
        plt.plot(*goal_lines)
    if plot_heading:
        plt.plot(*heading_lines)
    if plot_binds:
        plt.plot(*bind_lines)
    if plot_gps:
        plt.plot(gps_x, gps_y, 'g')
    if plot_kalman:
        plt.plot(kalman_x, kalman_y, 'b')


def plot_all(data_sets, map_name, plot_type, x_lim=None, y_lim=None,
             plot_map=True, plot_gps=True, plot_kalman=True,
             plot_heading=True, plot_binds=True, plot_goals=True,
             initial_gps=None):
    for data_set in data_sets:
        test_system(data_set, map_name, plot_type, x_lim, y_lim,
                    plot_map, plot_gps, plot_kalman,
                    plot_heading, plot_binds, plot_goals,
                    initial_gps)
        print(data_set + " finished")
    print("Done!")


def plot_gps(data_set):
    timestamps, data, length = get_data(data_set, ["gps long", "gps lat"])

    plt.plot(data[0], data[1])


def plot_map(map_name):
    map = Map(map_name)

    plt.plot(map[:, 0], map[:, 1], 'p')


if __name__ == '__main__':
    print(__doc__)
    test_system(
        "Test Day 11/Sat Apr 23 17;42;47 2016.csv",
        # "random",
        "wtracks map converted.csv",

        "kalman",

        plot_map=False, plot_gps=True, plot_kalman=True,
        plot_heading=False, plot_binds=True, plot_goals=False,
        # initial_gps=0
    )

    # test_with_servo(
    #     # "Test Day 9/Thu Apr 21 22;45;05 2016.csv",
    #     # "Test Day 7/Tue Apr 19 22;58;26 2016.csv",
    #     "Test Day 11/Sat Apr 23 15;41;11 2016.csv",
    #     "wtracks map converted.csv",
    #     plot_map=True, plot_kalman=False,
    #     plot_heading=True, plot_binds=True, plot_goals=False,
    #     enable_servo=True
    # )

    # plot_all(
    #     # ["random"] * 10,
    #
    #     ["Test Day 9/Thu Apr 21 22;34;11 2016.csv",
    #      # "Test Day 5/Sun Apr 10 18;21;57 2016.csv",
    #      "Test Day 6/Mon Apr 18 21;50;17 2016.csv",
    #      "Test Day 8/Wed Apr 20 21;51;46 2016.csv",
    #      "Test Day 8/Wed Apr 20 22;06;01 2016.csv",
    #      "Test Day 6/Mon Apr 18 22;19;40 2016.csv",
    #      # "Test Day 5/Sun Apr 10 18;30;08 2016.csv",
    #      # "Test Day 9/Thu Apr 21 22;14;05 2016.csv",
    #      # "Test Day 5/Sun Apr 10 18;21;57 2016.csv",
    #      ],
    #     "Trimmed Tue Apr 19 22;47;21 2016 GPS Map.csv",
    #     "kalman",
    #     plot_map=False, plot_gps=False, plot_kalman=True, plot_heading=True,
    #     plot_binds=False, plot_goals=False,
    #     initial_gps=0
    # )

    # plot_gps("Test Day 7/Tue Apr 19 22;31;53 2016.csv")
    # plot_gps("Test Day 7/Tue Apr 19 22;47;21 2016.csv")
    # plot_gps("Test Day 9/Thu Apr 21 23;04;06 2016.csv")
    # plot_gps("Test Day 6/Mon Apr 18 22;19;40 2016.csv")
    # plot_gps("Test Day 6/Mon Apr 18 21;50;17 2016.csv")

    # plot_map("Sat Feb 27 21;46;23 2016 Track Field Map.csv")
    # plot_map("Trimmed Minimalist Map.csv")
    # plot_map("Trimmed Tue Apr 19 22;47;21 2016 GPS Map.csv")
    # plot_map("wtracks map converted.csv")

    plt.show()
