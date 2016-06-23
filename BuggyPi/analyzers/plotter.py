import os
import sys

from matplotlib import pyplot as plt

sys.path.insert(0, "../")

import directories
from microcontroller.logger import Parser


def plot_gps(file_name, directory=None):
    lat, long = [], []
    parser = Parser(file_name, directory)
    for data in parser:
        if data[1] == 'gps':
            lat.append(data[2]['lat'])
            long.append(data[2]['long'])
    plt.plot(long, lat)
    plt.show()


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
    plt.show()


def plot_vs_time(file_name, name, value=None, directory=None):
    plot_data = []
    t = []
    parser = Parser(file_name, directory)
    for data in parser:
        if data[1] == name:
            plot_data.append(data[2][value])
            t.append(data[0])
    plt.plot(t, plot_data)
    plt.show()

file_name = "Wed Jun 22 20;39;21 2016"

# plot_vs_time(file_name, 'imu', 'yaw', directory="Jun 22 2016")
# plot_vs_time(file_name, 'encoder', 'counts', directory="Jun 22 2016")
plot_gps(file_name, "Jun 22 2016")

# parser = Parser("Wed Jun 22 20;59;40 2016")
#
# for data in parser:
#    if data[1] == 'imu':
#        print(data)

