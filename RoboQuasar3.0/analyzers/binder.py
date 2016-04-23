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

        self.map_dists = [0] * len(self.map)

    def bind(self, position):
        for index in range(len(self.map_dists)):
            dx = self.map[index][0] - position[0]
            dy = self.map[index][1] - position[1]
            self.map_dists[index] = math.sqrt(dx * dx + dy * dy)
        index = self.map_dists.index(
            min(self.map_dists))  # index of shortest distance

        bind_x, bind_y = self.map[index]
        goal_x, goal_y = self.map[(index + 1) % len(self.map)]
        return bind_x, bind_y, goal_x, goal_y

    def reshift(self, gps_long, gps_lat):
        self.map.reshift_map(gps_long, gps_lat)
