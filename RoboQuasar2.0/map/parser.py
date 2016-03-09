# handles map reading and finds goal position based on a supplied current position

import time
import csv
import sys
import os
import numpy as np
import copy

sys.path.insert(0, '../')

import config

class Map():
    def __init__(self, map_name, shift_angle, origin_lat, origin_long, directory=None):
        self.data = self.get_map(map_name, directory)
        self.origin_lat  = origin_lat * np.pi / 180
        self.origin_long = origin_long * np.pi / 180
        self.shift_matrix = np.array(
            [[np.cos(shift_angle), np.sin(shift_angle)],
             [-np.sin(shift_angle), np.cos(shift_angle)]]
        )
        self.deg_to_m = 111226.343

        self.convert_data()
        self.data = np.dot(self.data, self.shift_matrix)

    def convert_data(self):
        """
        goes through every lat, long pair and replaces it with the x y distance
        from the origin point

        :return: None
        """
        for i in range(len(self.data)):
            point_lat  = self.data[i][0] * np.pi / 180
            point_long = self.data[i][1] * np.pi / 180
            lat_mean = (point_lat + self.origin_lat) / 2
            x = (point_long - self.origin_long) * np.cos(lat_mean)
            y = (point_lat - self.origin_lat)
            self.data[i][0] = x * self.deg_to_m
            self.data[i][1] = y * self.deg_to_m


    @staticmethod
    def get_map(map_name, directory=None):
        if directory is None:
            directory = config.get_dir(":maps")

        with open(directory + map_name, 'r') as csvfile:
            map_reader = csv.reader(csvfile, delimiter=',', quotechar='|')
            parsed = [[float(row[0]), float(row[1])] for row in map_reader]
            return np.array(parsed)

    @staticmethod
    def write_map(data, directory=None, file_name=None):
        if directory is None:
            directory = config.get_dir(":maps")
        if file_name is None:
            file_name = time.strftime("%c").replace(":", ";")
        with open(directory + file_name + ".csv", 'w') as csv_file:
            map_writer = csv.writer(csv_file, delimiter=',',
                                    quotechar='|',
                                    quoting=csv.QUOTE_MINIMAL)
            for row in data:
                assert len(row) == 2
                map_writer.writerow(row)

    @staticmethod
    def remove_duplicates(map, write_output=True, directory=None, map_name=None):
        map = np.array(map)

        duplicates = np.where(np.diff(map, axis=0) == 0)

        map = np.delete(map, duplicates[0], 0)
        if write_output:
            write_map(map, directory, map_name)
        return map

    @staticmethod
    def convert_gpx(file_name, directory=None):
        if directory is None:
            directory = config.get_dir(":gpx")

        with open(directory + file_name, 'r') as gpx_file:
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

                data = [latitude, longitude]

                contents = contents[lon_index_end:]

            file_name = file_name[:-4] + " converted"
            write_map(data, directory, file_name)

class Binder:
    def __init__(self, map_name, shift_angle, origin_lat, origin_long, directory=None):
        self.map = Map(map_name, shift_angle, origin_lat, origin_long, directory)
        # set to None so that it will be able to start at any point on the track
        self.prev_bind = None

    def bind(self, position):
        if (self.prev_bind is None or self.prev_bind >= (len(self.map.data) - 1)
                or self.prev_bind < 0):
            # finds the smallest distance between the point and the map
            self.prev_bind = self.find_nearest(position)
            return self.map.data[self.prev_bind + 1]

        for index in range(self.prev_bind, len(self.map.data)):
            if self.is_near(index, position):
                self.prev_bind = index
                return self.map.data[index + 1]

        for index in range(self.prev_bind):
            if self.is_near(index, position):
                self.prev_bind = index
                return self.map.data[index + 1]

        self.prev_bind = self.find_nearest(position)
        return self.map.data[self.prev_bind + 1]

    def find_nearest(self, position):
        map_dist = [0] * len(self.map.data)
        for index in range(len(map_dist)):
            dlat = abs(float(self.map.data[index][0] - position[0]))
            dlong = abs(float(self.map.data[index][1] - position[1]))
            dist = ((dlat ** 2) + (dlong ** 2)) ** 0.5
            map_dist[index] = dist
        smallest_value = min(map_dist)
        index = map_dist.index(smallest_value)
        return index

    def is_near(self, index, position):
        dx = abs(float(self.map.data[index][0]) - position[0])
        dy = abs(float(self.map.data[index][1]) - position[1])
        dist = ((dx ** 2) + (dy ** 2)) ** 0.5

        if index + 2 < len(self.map.data):
            acc_dlat = abs(float(self.map.data[index][0] - self.map.data[index + 1][0]))
            acc_dlong = abs(float(self.map.data[index][1] - self.map.data[index + 1][1]))

        else:
            acc_dlat = abs(float(self.map.data[index][0] - self.map.data[index - 1][0]))
            acc_dlong = abs(float(self.map.data[index][1] - self.map.data[index - 1][1]))

        accuracy = ((acc_dlat ** 2 + acc_dlong ** 2) ** 0.5) / 2

        return dist <= accuracy


def binder_test():
    binder = Binder("Mon Mar  7 17;54;59 2016 GPS Map.csv", 0,
        26.611000061, 56.40650177)
    for row in binder.map.data:
        print("%f,%f" % (row[0], row[1]))

    pos1 = (26.6, 56.329)
    pos2 = (26.594, 56.34)
    pos3 = (26.59, 56.35)
    pos4 = (26.584, 56.357)
    pos5 = (26.582, 56.36)

    pos = binder.bind(pos1)
    pre_bind = binder.prev_bind
    print((binder.map.data[pre_bind]), "map[pre_bind]")
    print((pos, pre_bind), "pos, pre_bind")

    pos = binder.bind(pos2)
    pre_bind = binder.prev_bind
    print((binder.map.data[pre_bind]), "map[pre_bind]")
    print((pos, pre_bind), "pos, pre_bind")

    pos = binder.bind(pos3)
    pre_bind = binder.prev_bind
    print((binder.map.data[pre_bind]), "map[pre_bind]")
    print((pos, pre_bind), "pos, pre_bind")

    pos = binder.bind(pos4)
    pre_bind = binder.prev_bind
    print((binder.map.data[pre_bind]), "map[pre_bind]")
    print((pos, pre_bind), "pos, pre_bind")

    pos = binder.bind(pos5)
    pre_bind = binder.prev_bind
    print((binder.map.data[pre_bind]), "map[pre_bind]")
    print((pos, pre_bind), "pos, pre_bind")

def test_shift():
    test_obj = Map("2015-11-08 07_58_30.csv", 0,
                   40.4405106744, -79.9425712322)
    for row in test_obj.data:
        print("%f,%f" % (row[0], row[1]))

if __name__ == '__main__':
    # test_shift()
    binder_test()
