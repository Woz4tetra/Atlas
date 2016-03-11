"""
Written by Ben Warwick

data.py, written for RoboQuasar3.0
Version 3/10/2015
=========

Handles map reading and parsing
"""

import time
import csv
import sys
import os
import numpy as np
import copy

sys.path.insert(0, '../')

import config

class Map():
    def __init__(self, map_name=None, directory=None):
        if map_name is None:
            self.data = []
        else:
            self.data = self.get_map(map_name, directory)

    def __getitem__(self, item):
        return self.data[item]

    def __setitem__(self, item, value):
        self.data[item] = value

    @staticmethod
    def get_map(map_name, directory=None):
        if directory is None:
            directory = config.get_dir(":maps")

        with open(directory + map_name, 'r') as csvfile:
            map_reader = csv.reader(csvfile, delimiter=',', quotechar='|')
            parsed = []
            for row in map_reader:
                assert len(row) == 2
                parsed.append([float(row[0]), float(row[1])])
            return parsed

    def write_map(self, directory=None, file_name=None):
        if directory is None:
            directory = config.get_dir(":maps")
        if file_name is None:
            file_name = time.strftime("%c").replace(":", ";")
        with open(directory + file_name + ".csv", 'w') as csv_file:
            map_writer = csv.writer(csv_file, delimiter=',',
                                    quotechar='|',
                                    quoting=csv.QUOTE_MINIMAL)
            for row in self.data:
                assert len(row) == 2
                map_writer.writerow(row)

    def remove_duplicates(self, write_output=True, directory=None, map_name=None):
        data = np.array(self.data)

        duplicates = np.where(np.diff(data, axis=0) == 0)

        data = np.delete(data, duplicates[0], 0)
        self.data = data.tolist()
        if write_output:
            self.write_map(directory, map_name)
        return data

def convert_gpx(file_name, in_directory=None, out_directory=None):
    if in_directory is None:
        in_directory = config.get_dir(":gpx")

    if out_directory is None:
        out_directory = config.get_dir(":maps")

    with open(in_directory + file_name, 'r') as gpx_file:
        contents = gpx_file.read()
        data = []

        while len(contents) > 2:
            lat_index_start = contents.find("lat") + 5
            lat_index_end = contents.find('"', lat_index_start)
            latitude = contents[lat_index_start: lat_index_end]

            contents = contents[lat_index_end:]

            lon_index_start = contents.find("lon") + 5
            lon_index_end = contents.find('"', lon_index_start)
            longitude = contents[lon_index_start: lon_index_end]

            if len(longitude) > 0 and longitude[0] == '-':
                longitude = longitude[1:]
            data.append([latitude, longitude])

            contents = contents[lon_index_end:]
        data.pop(-1)

        file_name = file_name[:-4] + " converted"

        if "/" in file_name:
            dir_index = file_name.find("/")
            file_name = file_name[dir_index:]
        map = Map()
        map.data = data
        map.write_map(out_directory, file_name)


if __name__ == '__main__':
    pass
