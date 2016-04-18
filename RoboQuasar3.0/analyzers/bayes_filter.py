"""
Written by Ben Warwick

bayes_filter.py, written for RoboQuasar3.0
Version 3/10/2015
=========

A probably terrible implementation of the bayesian filter. Optimized ONLY for
the RoboQuasar system. This module will determine RoboQuasar's most likely
position on the provided gps map
"""

import math
import sys

import numpy as np
from scipy.ndimage import filters

sys.path.insert(0, "../")

from analyzers.map import Map


def old_round(number, decimal=0):
    place = 10 ** decimal
    return float(
        math.floor((number * place) + math.copysign(0.5, number))) / place


class BayesFilter():
    def __init__(self, map_name, initial_pos, num_redistrib_samples=None,
                 particle_cut_off=0.2, deg_to_meters=111226.343,
                 num_samples=None,
                 gauss_mask=(5, 0.5)):
        self.map = Map(map_name)  # GPS map of a course

        if num_samples is None:
            self.num_samples = len(self.map)
        else:
            self.num_samples = num_samples

        if num_redistrib_samples is None:
            self.redistrib_samples = self.num_samples // 3
        else:
            self.redistrib_samples = num_redistrib_samples

        self.error_slope = (len(self.map) - 1) / self.num_samples

        assert self.redistrib_samples < self.num_samples
        assert len(self.map) >= self.num_samples
        assert 0 < particle_cut_off <= 1.0

        self.particle_cut_off = particle_cut_off

        # Uniform sampling of num_samples points from the course
        # a list of indices from the map
        self.particles = [0] * self.num_samples
        increment = self.error_slope  # happens to be the same value
        map_index = 0
        for index in range(self.num_samples):
            self.particles[index] = int(map_index)
            map_index += increment

        # the weight of each particle (smaller numbers are better here)
        self.particle_weights = [1] * len(self.particles)

        self.deg_to_meters = deg_to_meters

        self.prev_pos = self.map.find_nearest(initial_pos)

        self.gauss_mask = self.gauss((gauss_mask[0], 1), gauss_mask[1])

        self.moved_by = 0

    def take_measurement(self, gps_coord):
        meas_lat, meas_lon = gps_coord
        self.prev_pos = self.map.find_nearest(gps_coord)

        for index in range(len(self.particles)):
            lat, lon = self.map[self.particles[index]]

            # weights determined by distance from measurement
            # Smaller distances are better (weighted higher)
            self.particle_weights[index] = ((lat - meas_lat) ** 2 + (
                lon - meas_lon) ** 2) ** 0.5
        likely_weight = min(self.particle_weights)
        # pprint.pprint(self.particle_weights)
        # print(min(self.particle_weights))
        locations = []
        for index, weight in enumerate(self.particle_weights):
            if abs(weight - likely_weight) < 0.000001:  # almost equal
                locations.append(self.map[self.particles[index]])
                # print(locations, weight, index)
        if len(locations) == 1:
            return locations[0]
        else:
            return locations


    def redistrib_particles(self):
        unlikely_weight = max(self.particle_weights)

        for index, weight in enumerate(self.particle_weights):
            if self.particle_cut_off > weight / unlikely_weight:
                self.particles[index] = self.particles[index]
            else:
                self.particles[index] = -1

            index_range = -weight / self.error_slope
            index + index_range, index - index_range

        # particles_left = self.num_samples - self.redistrib_samples
        # while particles_left > 0 and len(self.particles) != self.num_samples:
        #     random_index = np.random.randint(0, len(self.map))
        #     if random_index not in self.particles:
        #         self.particles.append(random_index)
        #         particles_left -= 1
        # assert len(self.particles) == self.num_samples

    @staticmethod
    def gauss(shape, nsig):
        structure = np.zeros(shape)
        structure[shape[0] // 2, shape[1] // 2] = 1
        if shape[1] == 1:
            return filters.gaussian_filter(structure, nsig)[:, 0]
        else:
            return filters.gaussian_filter(structure, nsig)

    def transition(self, enc_position):
        """
        Shift and spread the particles out based on how much RoboQuasar moved.
        enc_position is the current position on the map as determined by the
        encoder and IMU (IMU used for heading, make sure it's shifted to be
        relative to the map, use GPS heading). enc_position should be reset
        at the end of each time step to the GPS's measured position to keep
        it from drifting.
        """
        self.redistrib_particles()

        self.moved_by = self.map.find_nearest(enc_position) - self.prev_pos

        min_index = self.particles.index(min(self.particles))
        self.particles = self.particles[min_index:] + self.particles[:min_index]

        self.spread_particles()

    def spread_particles(self):
        for index in range(1, len(self.particles) // 2):
            dist = self.particles[index] + self.particles[index - 1]
            self.particles[index] += int(dist * old_round(self.moved_by / 2))
        for index in range(len(self.particles) - 1, len(self.particles) // 2, -1):
            dist = self.particles[index] + self.particles[index - 1]
            self.particles[index] -= int(dist * old_round(self.moved_by / 2))

        # shift indices by 'moved_by'
        for index in range(len(self.particles)):
            self.particles[index] = (self.moved_by + self.particles[index]) % len(
                self.map)

def test_line():
    bayes = BayesFilter("test maps/test map line.csv", (0, 0), num_samples=10)
    assert bayes.take_measurement((0.5, 0.5)) == [0, 0]

    # a previous measurement done without a follow up transition will be
    # overwritten by the next measurement
    assert bayes.take_measurement((0.55, 0.5)) == [0, 0]
    assert bayes.take_measurement((20, 0)) == [21, 0]
    assert bayes.take_measurement((0, 0)) == [0, 0]

    print(bayes.particles)
    # bayes.transition((1.1, 0.02))
    bayes.redistrib_particles()
    print(bayes.particles)

def test_circle():
    bayes = BayesFilter("test maps/test map circle.csv", (11, 0))
    # print(bayes.take_measurement((10.5, 0.1)))
    # bayes.transition((10.3, 1.8))
    # print(bayes.take_measurement((10.3480775301221, 1.8364817766693)))
    # bayes.transition((9.3, 1))
    # print(bayes.take_measurement((9.89692620785909, 3.52020143325669)))
    print(bayes.take_measurement((10, 0)))
    bayes.transition((11, 0.8))
    print(bayes.take_measurement((10.2, 3)))
    bayes.transition((11.01, 0.81))
    print(bayes.take_measurement((10.2, 3)))
    bayes.transition((11.02, 0.82))
    print(bayes.take_measurement((10.2, 3)))

if __name__ == '__main__':
    test_line()