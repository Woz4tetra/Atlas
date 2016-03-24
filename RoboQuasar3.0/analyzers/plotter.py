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
from analyzers.logger import parse


def get_plottable_data(file_name, sensors):
    data = parse(file_name, np_array=False, omit_header_rows=False)

    columns = []
    for sensor_name in sensors:
        columns.append(data[1].index(sensor_name))
    data.pop(0)
    data.pop(0)

    data = np.array(data)

    timestamps = data[:, 0]
    sensor_data = []
    for column in columns:
        sensor_data.append(data[:, column])

    return timestamps, sensor_data


def plot_gps(file_name):
    timestamps, sensor_data = get_plottable_data(file_name, ["lat", "long"])
    print(sensor_data)

    plt.plot(sensor_data[0], sensor_data[1])

    axes = plt.gca()
    axes.set_xlim([40.4425, 40.444])
    axes.set_ylim([79.939, 79.942])


def plot_heading(file_name):
    timestamps, sensor_data = get_plottable_data(file_name, ["yaw", "compass"])

    plt.plot(timestamps, sensor_data[0], timestamps, sensor_data[1])


def plot_encoder(file_name):
    timestamps, sensor_data = get_plottable_data(file_name, ["counts"])

    plt.plot(timestamps, sensor_data[0])


def plot_angles(file_name, density=100, source="yaw", set_limits=True,
                data_range=None, shift_angle=0.0, swap_angle=False,
                rainbow_colors=False):
    if source == "atan":
        timestamps, sensor_data = get_plottable_data(file_name,
                                                     ["lat", "long"])
    else:
        # lat: x, long: y, source: angle
        timestamps, sensor_data = get_plottable_data(file_name,
                                                     ["lat", "long", source])
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
    data = parse(config.get_dir(":maps") + map_name, omit_header_rows=False)
    plt.plot(data[:, 0], data[:, 1])


if __name__ == '__main__':
    # plot_all(plot_angles, source="yaw", directory="Test Day 4", density=50)
    # plot_angles("Test Day 4/Sat Mar 12 22;27;45 2016.csv", source="yaw",
    #             density=100, shift_angle=np.pi / 2, swap_angle=True)
    # plot_angles("Autonomous Test Day 2/parsed2.csv", source="atan",
                # density=100)

    # plot_encoder("Test Day 4/Sat Mar 12 23;06;53 2016.csv")
    plot_gps("Autonomous Test Day 2/parsed2.csv")

    # plot_map("Track Field Map Trimmed.csv")

    plt.show()
