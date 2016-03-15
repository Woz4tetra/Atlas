"""
Written by Ben Warwick

plotter.py, written for RoboQuasar3.0
Version 3/10/2015
=========

Visualizes data retrieved by logger.py
"""

from matplotlib import pyplot as plt
import numpy as np
import sys
import os

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

    plt.plot(sensor_data[0], sensor_data[1])

    axes = plt.gca()
    axes.set_xlim([40.4425,40.444])
    axes.set_ylim([79.939,79.942])

def plot_heading(file_name):
    timestamps, sensor_data = get_plottable_data(file_name, ["yaw", "compass"])

    plt.plot(timestamps, sensor_data[0], timestamps, sensor_data[1])

def plot_encoder(file_name):
    timestamps, sensor_data = get_plottable_data(file_name, ["counts"])

    plt.plot(timestamps, sensor_data[0])

def plot_angles(file_name, density=100, source="yaw"):
    timestamps, sensor_data = get_plottable_data(file_name, ["lat", "long", source])

    plt.plot(sensor_data[0], sensor_data[1])

    lines = []
    for index in range(0, len(sensor_data[0]), density):
        lines.append(
            (sensor_data[0][index],
             sensor_data[0][index] + 0.0001 * np.sin(sensor_data[2][index]))
        )
        lines.append(
            (sensor_data[1][index],
             sensor_data[1][index] + 0.0001 * np.cos(sensor_data[2][index]))
        )
        # lines.append("#%0.2x%0.2x%0.2x" %
        #     (255 - int(np.random.normal(128, 128)) % 255,
        #      255 - int(np.random.normal(128, 128)) % 255,
        #      255 - int(np.random.normal(128, 128)) % 255))
        lines.append('r')
    plt.plot(*lines)

    axes = plt.gca()
    axes.set_xlim([40.4425,40.444])
    axes.set_ylim([79.939,79.942])

def plot_all(plot_func, directory=None, **params):
    if not os.path.isdir(directory):
        directory = config.get_dir(":logs") + directory

    if directory[-1] != "/":
        directory += "/"

    for file_name in os.listdir(directory):
        if file_name.endswith(".csv"):
            print(directory + file_name)
            plot_func(directory + file_name, **params)

if __name__ == '__main__':
    plot_all(plot_angles, source="yaw", directory="Test Day 4")
    # plot_angles("Test Day 4/Sat Mar 12 23;06;53 2016.csv", density=100)
    # plot_encoder("Test Day 4/Sat Mar 12 23;06;53 2016.csv")

    # data = parse("/home/atlas/Documents/Atlas/RoboQuasar2.0/map/maps/2016-03-07 21_46_18 converted.csv", omit_header_rows=False)
    # plt.plot(data[:, 0], data[:, 1])

    plt.show()
