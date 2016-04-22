"""
Written by Ben Warwick

plotter.py, written for RoboQuasar3.0
Version 3/10/2015
=========

Visualizes data retrieved by logger.py
"""

import os
import sys

import numpy as np
from matplotlib import pyplot as plt

sys.path.insert(0, "../")

import config
from analyzers.logger import get_data, parse
from analyzers.binder import Binder


def plot_gps(file_name):
    timestamps, sensor_data = get_data(file_name, ["gps_lat", "gps_long"])
    print(sensor_data)

    plt.plot(sensor_data[0], sensor_data[1])

    axes = plt.gca()
    axes.set_xlim([40.4425, 40.444])
    axes.set_ylim([79.939, 79.942])


def plot_heading(file_name):
    timestamps, sensor_data = get_data(file_name, ["yaw", "compass"])

    plt.plot(timestamps, sensor_data[0], timestamps, sensor_data[1])


def plot_encoder(file_name):
    timestamps, sensor_data = get_data(file_name, ["counts"])

    plt.plot(timestamps, sensor_data[0])


def plot_angles(file_name, density=100, source="yaw", set_limits=True,
                data_range=None, shift_angle=0.0, swap_angle=False,
                rainbow_colors=False):
    if source == "atan":
        timestamps, sensor_data = get_data(file_name,
                                           ["gps_lat", "gps_long"])
    else:
        # lat: x, long: y, source: angle
        timestamps, sensor_data = get_data(file_name,
                                           ["gps_lat", "gps_long", source])
    if data_range is None:
        data_range = (0, len(sensor_data[0]) - density)
    else:
        data_range = (data_range[0], data_range[1] - density)

    plt.plot(sensor_data[0][data_range[0]: data_range[1]],
             sensor_data[1][data_range[0]: data_range[1]])

    lines = []
    for index in range(data_range[0], data_range[1], density):
        x1, x0 = sensor_data[0][index + density], sensor_data[0][index]
        y1, y0 = sensor_data[1][index + density], sensor_data[1][index]
        dx = x1 - x0
        dy = y1 - y0

        # hypotenuse of a triangle for coords, 1.5 is to have it stick out more
        hypotenuse = 1.5 * ((dx ** 2 + dy ** 2) ** .5)

        if source == "atan":
            angle = np.arctan2(dy, dx)
        else:
            angle = sensor_data[2][index]

        if swap_angle:
            angle = shift_angle - angle
        else:
            angle += shift_angle

        lines.append(
            (sensor_data[0][index],
             sensor_data[0][index] + hypotenuse * np.cos(angle))
        )
        lines.append(
            (sensor_data[1][index],
             sensor_data[1][index] + hypotenuse * np.sin(angle))
        )
        if rainbow_colors:
            lines.append("#%0.2x%0.2x%0.2x" %
                (255 - int(np.random.normal(128, 128)) % 255,
                 255 - int(np.random.normal(128, 128)) % 255,
                 255 - int(np.random.normal(128, 128)) % 255))
        else:
            lines.append('r')

    plt.plot(*lines)

    if set_limits:
        axes = plt.gca()
        axes.set_xlim([40.4425, 40.444])
        axes.set_ylim([79.939, 79.942])


def plot_all(plot_func, directory=None, **params):
    if not os.path.isdir(directory):
        directory = config.get_dir(":logs") + directory

    if directory[-1] != "/":
        directory += "/"

    for file_name in os.listdir(directory):
        if file_name.endswith(".csv"):
            print(directory + file_name)
            plot_func(directory + file_name, **params)


def plot_map(map_name):
    data = parse(config.get_dir(":maps") + map_name, omit_header_row=False)
    plt.plot(data[:, 0], data[:, 1])


def plot_kalman(file_name, map_name, directory=""):
    if not os.path.isdir(directory):
        directory = config.get_dir(":logs") + directory

    if directory[-1] != "/":
        directory += "/"

    timestamps, data, length = get_data(directory + file_name,
        ["kalman x", "kalman y", "kalman heading", "bind x", "bind y", "gps long", "gps lat"],
        density=100)
    kalman_x, kalman_y, kalman_heading, bind_x, bind_y, gps_long, gps_lat = data

    binder = Binder(map_name, gps_long[0], gps_lat[0])

    x, y = [], []
    for index in range(len(binder.map.data)):
        x.append(binder.map.data[index][0])
        y.append(binder.map.data[index][1])
    plt.plot(x, y)

    x, y = [], []
    for index in range(length):
        x.append(kalman_x[index])
        y.append(kalman_y[index])
    plt.plot(x, y)

    lines = []
    for index in range(length):
        lines.append((kalman_x[index], kalman_x[index] + 10 * np.cos(kalman_heading[index])))
        lines.append((kalman_y[index], kalman_y[index] + 10 * np.sin(kalman_heading[index])))
        lines.append('r')
    plt.plot(*lines)

    lines2 = []
    for index in range(length):
        lines2.append((kalman_x[index], bind_x[index]))
        lines2.append((kalman_y[index], bind_y[index]))
        lines2.append('g')
    plt.plot(*lines2)

if __name__ == '__main__':
    # plot_angles("Test Day 5/Sun Apr 10 18;30;08 2016.csv", source="yaw",
    #             density=1000)
    # plot_kalman("Test Day 7/Tue Apr 19 22;58;26 2016.csv", "Tue Apr 19 22;47;21 2016 GPS Map.csv")
    plot_map("Tue Apr 19 22;47;21 2016 GPS Map.csv")

    plt.show()
