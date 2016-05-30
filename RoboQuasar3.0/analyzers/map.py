"""
Written by Ben Warwick

data.py, written for RoboQuasar3.0
Version 3/10/2015
=========

Handles map reading and parsing
"""

import csv
import math
import pprint
import sys
import time

import numpy as np

sys.path.insert(0, '../')

import config


class Map():
    def __init__(self, map_name=None, directory=None, origin_long=None,
                 origin_lat=None):
        if map_name is None:
            self.raw_data = []
            self.data = []
        else:
            self.raw_data = np.array(self.get_map(map_name, directory))
            self.earth_radius = 6371000

            if origin_lat is not None and origin_long is not None:
                self.origin_long = origin_long
                self.origin_lat = origin_lat
            else:
                self.origin_long = self.raw_data[0][0]
                self.origin_lat = self.raw_data[0][1]
            self.data = np.array(self.shift_data())

    def convert_gps(self, prev_long, prev_lat, longitude, latitude):
        phi1 = math.radians(prev_lat)
        phi2 = math.radians(latitude)

        lam1 = math.radians(prev_long)
        lam2 = math.radians(longitude)

        dphi = math.radians(latitude - prev_lat)
        dlam = math.radians(longitude - prev_long)

        a = math.sin(dphi / 2) ** 2 + \
            math.cos(phi1) * math.cos(phi2) * \
            math.sin(dlam / 2) * math.sin(dlam / 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        dist = self.earth_radius * c

        y = math.sin(lam2 - lam1) * math.cos(phi2)
        x = math.cos(phi1) * math.sin(phi2) - \
            math.sin(phi1) * math.cos(phi2) * math.cos(lam2 - lam1)
        bearing = math.atan2(y, x)

        return dist * math.cos(bearing), dist * math.sin(bearing), bearing

    def shift_data(self):
        """
        goes through every lat, long pair and replaces it with the x y distance
        from the origin point

        :return: None
        """

        data = []

        for i in range(len(self.raw_data)):
            x, y, bearing = self.convert_gps(self.origin_long, self.origin_lat,
                                             self.raw_data[i][0],
                                             self.raw_data[i][1])
            data.append([x, y])

        return data

    def reshift_map(self, new_long, new_lat):
        self.origin_long = new_long
        self.origin_lat = new_lat
        self.data = np.array(self.shift_data())

    def __getitem__(self, item):
        return self.data[item]

    def __setitem__(self, item, value):
        self.data[item] = value

    def __len__(self):
        return len(self.data)

    def __str__(self):
        return pprint.pformat(self.data)

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

    def write_map(self, directory=None, file_name=None, use_xy=False):
        if directory is None:
            directory = config.get_dir(":maps")
        if file_name is None:
            file_name = time.strftime("%c").replace(":", ";")
        if not file_name.endswith(".csv"):
            file_name += ".csv"
        print("Writing to: " + directory)
        print("File name to: " + file_name)

        if use_xy:
            map_data = self.data
        else:
            map_data = self.raw_data

        with open(directory + file_name, 'w') as csv_file:
            map_writer = csv.writer(csv_file, delimiter=',',
                                    quotechar='|',
                                    quoting=csv.QUOTE_MINIMAL)
            for row in map_data:
                assert len(row) == 2
                map_writer.writerow(row)

    def remove_duplicates(self, write_output=True, directory=None,
                          map_name=None, use_xy=False):
        if use_xy:
            map_data = self.data
        else:
            map_data = self.raw_data
        data = np.array(map_data)

        duplicates = np.where(np.diff(data, axis=0) == 0)

        data = np.delete(data, duplicates[0], 0)
        if use_xy:
            self.data = data.tolist()
        else:
            self.raw_data = data.tolist()
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
            data.append([longitude, latitude])

            contents = contents[lon_index_end:]
        data.pop(-1)

        file_name = file_name[:-4] + " converted"

        if "/" in file_name:
            dir_index = file_name.find("/")
            file_name = file_name[dir_index:]
        map = Map()
        map.raw_data = data
        map.write_map(out_directory, file_name)


def make_map(log_file, map_name, sensors=("gps long", "gps lat")):
    from analyzers.logger import get_data
    timestamps, sensor_data, length = get_data(log_file, sensors)
    map_data = []
    for index in range(length):
        map_data.append([sensor_data[0][index], sensor_data[1][index]])
    map = Map()
    map.raw_data = map_data
    map.remove_duplicates(map_name=map_name)


def shift_map(map_name, new_name, origin_lat, origin_long):
    Map(map_name, origin_lat=origin_lat,
        origin_long=origin_long).remove_duplicates(map_name=new_name)

# if __name__ == '__main__':
#     Map("Tue Apr 19 22;47;21 2016 GPS Map.csv").remove_duplicates()
#     make_map("Test Day 8/Wed Apr 20 21;51;46 2016.csv", "Map from Wed Apr 20 21;51;46 2016.csv")
#     convert_gpx("Buggy Course/wtracks map.gpx")
