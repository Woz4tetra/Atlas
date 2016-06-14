import os
import sys

from matplotlib import pyplot as plt

sys.path.insert(0, "../")

import config
from microcontroller.logger import Parser


def plot_gps(file_name, directory):
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
    for file_name in os.listdir(config.get_dir(":logs") + directory):
        parser = Parser(file_name, directory)
        for data in parser:
            if data[1] == 'gps':
                if data[2]['lat'] > 0 and data[2]['long'] > 0:
                    lat.append(data[2]['lat'])
                    long.append(data[2]['long'])
        plt.plot(long, lat)
        lat, long = [], []
    plt.show()

# plot_gps("Mon Jun 13 21;23;34 2016", "Jun 13 2016")
plot_all_gps("Jun 13 2016")
