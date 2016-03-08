# handles map reading and finds goal position based on a supplied current position

import time
import csv
import sys
import os
import numpy as np
import copy
from scipy.spatial.distance import cdist as distance

sys.path.insert(0, '../')

import config


def get_map(directory=None):
    maps_dir = config.get_dir(":maps")
    if directory is None:
        for file_name in os.listdir(maps_dir):
            if len(file_name) >= 5 and file_name[-4:] == '.csv':
                directory = maps_dir + file_name
                break
        if directory is None:
            raise FileNotFoundError("No valid maps to choose from.")

    if not os.path.isdir(directory):
        directory = maps_dir + directory

    with open(directory, 'r') as csvfile:
        map_reader = csv.reader(csvfile, delimiter=',', quotechar='|')
        parsed = [[float(row[0]), float(row[1])] for row in map_reader]
        return parsed


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


def remove_duplicates(map, write_output=True, directory=None, map_name=None):
    map = np.array(map)

    duplicates = np.where(np.diff(map, axis=0) == 0)

    map = np.delete(map, duplicates[0], 0)
    if write_output:
        write_map(map, directory, map_name)
    return map


class Binder:
    def __init__(self, map_name):
        self.map = get_map(map_name)
        #set to None so that it will be able to start at any point on the track
        self.prev_bind = None

    def bind(self, position):
        if (self.prev_bind is None or self.prev_bind >= (len(self.map) - 1)
            or self.prev_bind < 0):
            #finds the smallest distance between the point and the map
            self.prev_bind = self.find_nearest(position)
            return self.map[self.prev_bind+1]

        for index in range(self.prev_bind, len(self.map)):
            if self.is_near(index, position):
                self.prev_bind = index
                return self.map[index+1]

        for index in range(self.prev_bind):
            if self.is_near(index, position):
                self.prev_bind = index
                return self.map[index+1]

        return False

    def find_nearest(self, position):
        map_dist = [0] * len(self.map)
        for index in range(len(map_dist)):
            dlat  = abs(float(self.map[index][0] - position[0]))
            dlong = abs(float(self.map[index][1] - position[1]))
            dist = ((dlat ** 2) + (dlong ** 2)) ** 0.5
            map_dist[index] = dist
        smallest_value = min(map_dist)
        index = map_dist.index(smallest_value)
        return index

    def is_near(self, index, position):
        dx = abs(float(self.map[index][0]) - position[0])
        dy = abs(float(self.map[index][1]) - position[1])
        dist = ((dx ** 2) + (dy ** 2)) ** 0.5


        if index + 2 < len(self.map):
            acc_dlat  = abs(float(self.map[index][0] - self.map[index+1][0]))
            acc_dlong = abs(float(self.map[index][1] - self.map[index+1][1]))

        else:
            acc_dlat = abs(float(self.map[index][0] - self.map[index - 1][0]))
            acc_dlong = abs(float(self.map[index][1] - self.map[index - 1][1]))

        accuracy = ((acc_dlat ** 2 + acc_dlong ** 2) ** 0.5)/2

        return dist <= accuracy


def test():
    pos1 = (26.6, 56.329)
    pos2 = (26.594, 56.34)
    pos3 = (26.59, 56.35)
    pos4 = (26.584, 56.357)
    pos5 = (26.582, 56.36)

    pos = binder.bind(pos1)
    pre_bind = binder.prev_bind
    print((binder.map[pre_bind]), "map[pre_bind]")
    print((pos, pre_bind), "pos, pre_bind")

    pos = binder.bind(pos2)
    pre_bind = binder.prev_bind
    print((binder.map[pre_bind]), "map[pre_bind]")
    print((pos, pre_bind), "pos, pre_bind")

    pos = binder.bind(pos3)
    pre_bind = binder.prev_bind
    print((binder.map[pre_bind]), "map[pre_bind]")
    print((pos, pre_bind), "pos, pre_bind")

    pos = binder.bind(pos4)
    pre_bind = binder.prev_bind
    print((binder.map[pre_bind]), "map[pre_bind]")
    print((pos, pre_bind), "pos, pre_bind")

    pos = binder.bind(pos5)
    pre_bind = binder.prev_bind
    print((binder.map[pre_bind]), "map[pre_bind]")
    print((pos, pre_bind), "pos, pre_bind")


if __name__ == '__main__':
    binder = Binder("Mon Mar  7 17;54;59 2016 GPS Map.csv")

    # pos = binder.bind((26.6156005859, 56.423500061))
    # pre_bind = binder.prev_bind
    # print((binder.map[pre_bind]), "map[pre_bind]")
    # print((pos, pre_bind), "pos, pre_bind")
    #
    # # binder = Binder("Mon Mar  7 17;54;59 2016 GPS Map.csv")
    #
    # pos = binder.bind((26.605298996, 56.478302002))
    # pre_bind = binder.prev_bind
    # print((binder.map[pre_bind]), "map[pre_bind]")
    # print((pos, pre_bind), "pos, pre_bind")

    test()
