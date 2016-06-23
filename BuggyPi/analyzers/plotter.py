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


plot_vs_time("Wed Jun 22 20;59;40 2016", 'servo')
# plot_all_gps("Jun 13 2016")
##parser = Parser("Wed Jun 22 20;59;40 2016")
##
##for data in parser:
##    if data[1] == 'imu':
##        print(data)
        
