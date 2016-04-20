"""
Written by Ben Warwick

data.py, written for RoboQuasar3.0
Version 3/10/2015
=========

Finds goal position based on a supplied current position
"""

import math
from analyzers.map import Map


class Binder:
    def __init__(self, map_name, origin_long=None, origin_lat=None):
        self.map = Map(map_name, origin_long=origin_long, origin_lat=origin_lat)
        # set to None so that it will be able to start at any point on the track
        self.prev_bind = None

        self.map_dists = [0] * len(self.map.data)

    def bind(self, position):
        for index in range(len(self.map_dists)):
            dx = self.map.data[index][0] - position[0]
            dy = self.map.data[index][1] - position[1]
            self.map_dists[index] = math.sqrt(dx * dx + dy * dy)
        index = self.map_dists.index(min(self.map_dists))  # index of shortest distance

        return self.map[(index + 1) % len(self.map)]

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
            acc_dlat = abs(
                float(self.map.data[index][0] - self.map.data[index + 1][0]))
            acc_dlong = abs(
                float(self.map.data[index][1] - self.map.data[index + 1][1]))

        else:
            acc_dlat = abs(
                float(self.map.data[index][0] - self.map.data[index - 1][0]))
            acc_dlong = abs(
                float(self.map.data[index][1] - self.map.data[index - 1][1]))

        accuracy = ((acc_dlat ** 2 + acc_dlong ** 2) ** 0.5) / 2

        return dist <= accuracy
