"""
Written by Ben Warwick

bayes_filter.py, written for RoboQuasar3.0
Version 3/10/2015
=========

A probably terrible implementation of the bayesian filter. Optimized ONLY for
the RoboQuasar system. This module will determine RoboQuasar's most likely
position on the provided gps map
"""

import numpy as np
from scipy.ndimage import filters
import math
import sys

sys.path.insert(0, "../")

from analyzers.map import Map

def old_round(number, decimal=0):
    place = 10 ** decimal
    return float(
        math.floor((number * place) + math.copysign(0.5, number))) / place


class BayesFilter():
    def __init__(self, map_name, initial_pos, num_redistrib_samples=None,
                 particle_cut_off=0.5, deg_to_meters=111226.343,
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

        self.particle_cut_off = particle_cut_off

        # Uniform sampling of num_samples points from the course
        # a list of indices from the map
        self.particles = [None] * self.num_samples
        increment = len(self.map) // self.num_samples
        map_index = 0
        for index in range(self.num_samples):
            self.particles[index] = map_index
            map_index += increment

        # the weight of each particle (smaller numbers are better here)
        self.particle_weights = [1] * len(self.particles)

        self.deg_to_meters = deg_to_meters

        self.prev_pos = self.find_nearest(initial_pos)

        self.gauss_mask = self.gauss((gauss_mask[0], 1), gauss_mask[1])

    def take_measurement(self, gps_coord):
        meas_lat, meas_lon = gps_coord
        self.prev_pos = self.find_nearest(gps_coord)

        for index in range(len(self.particles)):
            lat, lon = self.map[self.particles[index]]

            # weights determined by distance from measurement
            # Smaller distances are better (weighted higher)
            self.particle_weights[index] = ((lat - meas_lat) ** 2 + (
                lon - meas_lon) ** 2) ** 0.5

        likely_weight = min(self.particle_weights)
        locations = []
        for index, weight in enumerate(self.particle_weights):
            if weight == likely_weight:
                locations.append(self.map[self.particles[index]])
        return locations

    def redistrib_particles(self):
        unlikely_weight = max(self.particle_weights)
        self.particles = []

        for index, weight in enumerate(self.particle_weights):
            if self.particle_cut_off < weight / unlikely_weight:
                self.particles.append(index)

        particles_left = self.redistrib_samples
        major_end_index = len(self.particles)
        index = 0
        add_forward = True
        offset = 1
        while particles_left > 0 and len(self.particles) != self.num_samples:
            if add_forward:
                if self.particles[index] + offset not in self.particles:
                    self.particles.append(self.particles[index] + offset)
            else:
                if self.particles[index] - offset not in self.particles:
                    self.particles.append(self.particles[index] - offset)

            particles_left -= 1
            index = (index + 1) % major_end_index
            if index == 0:
                add_forward = not add_forward
                offset += 1

        particles_left = self.num_samples - self.redistrib_samples
        while particles_left > 0 and len(self.particles) != self.num_samples:
            random_index = np.random.randint(0, len(self.map))
            if random_index not in self.particles:
                self.particles.append(random_index)
                particles_left -= 1
        assert len(self.particles) == self.num_samples

    @staticmethod
    def gauss(shape, nsig):
        structure = np.zeros(shape)
        structure[shape[0] // 2, shape[1] // 2] = 1
        if shape[1] == 1:
            return filters.gaussian_filter(structure, nsig)[:, 0]
        else:
            return filters.gaussian_filter(structure, nsig)

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

        moved_by = self.find_nearest(enc_position) - self.prev_pos

        min_index = self.particles.index(min(self.particles))
        self.particles = self.particles[min_index:] + self.particles[:min_index]

        # spread particles (transition error)
        for index in range(1, len(self.particles) // 2):
            dist = self.particles[index] + self.particles[index - 1]
            self.particles[index] += int(dist * old_round(moved_by / 2))
        for index in range(len(self.particles) - 1, len(self.particles) // 2, -1):
            dist = self.particles[index] + self.particles[index - 1]
            self.particles[index] -= int(dist * old_round(moved_by / 2))

        # shift indices by 'moved_by'
        for index in range(len(self.particles)):
            self.particles[index] = (moved_by + self.particles[index]) % len(
                self.map)


if __name__ == '__main__':
    bayes = BayesFilter("test maps/test map circle.csv", (11, 0))
    print(bayes.take_measurement((10.5, 0.1)))
    bayes.transition((10.3, 1.8))
    print(bayes.take_measurement((10.3480775301221, 1.8364817766693)))
    bayes.transition((9.3, 1))
    print(bayes.take_measurement((9.89692620785909, 3.52020143325669)))
    # print(bayes.take_measurement((0, 0)))
