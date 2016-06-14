"""
Written by Ben Warwick

bayes_filter.py, written for RoboQuasar3.0
Version 3/10/2015
=========

A probably terrible implementation of the bayesian filter. Optimized ONLY for
the RoboQuasar system. This module will determine RoboQuasar's most likely
position on the provided gps map

There is no probabilistic redistribution of particles in this implementation
as of now. This implementation assumes that RoboQuasar is following the
gps map relatively closely as it binds resulting locations to the gps map.
"""

import math


def old_round(number, decimal=0):
    place = 10 ** decimal
    return int(float(
        math.floor((number * place) + math.copysign(0.5, number))) / place)


class PositionFilter:
    def __init__(self, binder):
        self.binder = binder

        # self.particles = [0] * len(binder.map)
        self.weights = [0] * len(binder.map)

        self.m_to_index = self.get_conversion()
        self.prev_index = 0

        self.dist = 0

    def get_conversion(self):
        # assuming map points are distributed relatively evenly
        sum_dist = 0
        prev_x, prev_y = 0, 0
        for x, y in self.binder.map:
            sum_dist += self.distance(x, y, prev_x, prev_y)
            prev_x = x
            prev_y = y
        print(len(self.binder.map), sum_dist)
        return len(self.binder.map) / sum_dist

    @staticmethod
    def distance(x0, y0, x1, y1):
        return math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)

    def take_measurement(self, gps_x, gps_y):
        likely_weight = None
        likely_locations = []
        for index in range(len(self.weights)):
            x, y = self.binder.map[index]
            self.weights[index] += self.distance(gps_x, gps_y, x, y)
            if likely_weight is None or self.weights[index] <= likely_weight:
                if self.weights[index] == likely_weight:
                    likely_locations.append((x, y))
                else:
                    likely_locations = [(x, y)]
                likely_weight = self.weights[index]
                # self.prev_index = index

        return likely_locations

    def transition(self, change_dist):
        self.dist += change_dist
        index = int(self.dist * self.m_to_index)

        if index - self.prev_index > 0:
            temp_weights = [0] * len(self.weights)
            for w_index, weight in enumerate(self.weights):
                temp_weights[((index - self.prev_index) + w_index) % len(self.weights)] = weight
            self.weights = temp_weights

        self.prev_index = index

        likely_weight = self.weights[0]
        likely_locations = []
        for index in range(len(self.weights)):
            if likely_weight is None or self.weights[index] <= likely_weight:
                if self.weights[index] == likely_weight:
                    likely_locations.append(tuple(self.binder.map[index]))
                else:
                    likely_locations = [tuple(self.binder.map[index])]
                likely_weight = self.weights[index]

        return likely_locations
