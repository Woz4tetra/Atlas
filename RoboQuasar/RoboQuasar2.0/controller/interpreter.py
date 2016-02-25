"""
    Written by Jason Kagie (modified by Ben Warwick)

    interpreter.py, written for RoboQuasar1.0
    Version 1/4/2016
    =========

    Interprets sensor data from board/data.py to x, y, theta (current position).

    Classes:
        Filter - contains a kalman filter tailored for gps, encoder,
            accelerometer and orientation input
"""

import numpy as np
import pykalman
import time
import math


class MainFilter(object):
    def __init__(self, latitude, longitude, circum, heading):
        self.headingKalman = headingKalman()
        self.placeKalman = placeKalman((latitude, longitude), circum, heading)
        self.time = time.time()
        self.dt = 0

    def update(self, gps_lat, gps_lon, encoder_counts, accel_x, accel_y, gyro_z, heading):
        self.dt = time.time() - self.time
        self.time = time.time()
        phi = self.headingKalman.update(heading, gyro_z, self.dt)
        x, y = self.placeKalman.update(gps_lat, gps_lon, encoder_counts, accel_x, accel_y, phi, self.dt)
        return x, y, phi


class headingKalman(object):
    def __init__(self):
        self.filter = pykalman.KalmanFilter()
        self.filt_state_mean = np.array([0.0, 0.0])  # phi0 = 0, Vang0 = 0
        self.covariance = np.identity(2)
        self.obs_matrix = np.identity(2)

    def update(self, heading, gyro_z, dt):
        trans_matrix = np.array([[1, dt],
                                 [0, 1]])
        obs = np.array([heading, gyro_z])
        self.filt_state_mean, self.covariance = self.filter.filter_update(
                filtered_state_mean=self.filt_state_mean,
                filtered_state_covariance=self.covariance,
                observation=obs,
                transition_matrix=trans_matrix,
                observation_matrix=self.obs_matrix)

        return self.filt_state_mean[0]

class placeKalman(object):
    def __init__(self, origin, circum, start_heading):
        self.origin = origin
        self.circum = circum
        self.filter = pykalman.KalmanFilter()
        self.filt_state_mean = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.covariance = np.identity(6)
        self.displacement_angle = start_heading

    def update(self, gps_lat, gps_lon, encoder_counts, accel_x, accel_y, phi, dt):
        (dist, x_gps, y_gps) = self.geo_dist(gps_lat, gps_lon)
        observation = np.array([x_gps, y_gps,
                                encoder_counts, accel_x, accel_y])
        x_coeff = 0
        y_coeff = 0
        if (math.cos(phi) == 0):
            y_coeff = self.circum*dt / math.sin(phi)
            x_coeff = 0
        elif (math.sin(phi) == 0):
            x_coeff = self.circum * dt / math.cos(phi)
            y_coeff = 0
        else:
            x_coeff = 0.5 * self.circum * dt / math.cos(phi)
            y_coeff = 0.5 * self.circum * dt / math.sin(phi)
        observation_matrix = np.array(
                [[1, 0, 0, 0, 0, 0],
                 [0, 1, 0, 0, 0, 0],
                 [0, 0, x_coeff, y_coeff, 0, 0],
                 [0, 0, 0, 0, 1, 0],
                 [0, 0, 0, 0, 0, 1]])

        transition_matrix = np.array(
                [[1, 0, dt, 0, 0, 0],
                 [0, 1, 0, dt, 0, 0],
                 [0, 0, 1, 0, dt, 0],
                 [0, 0, 0, 1, 0, dt],
                 [0, 0, 0, 0, 1, 0],
                 [0, 0, 0, 0, 0, 1]]
        )

        self.filt_state_mean, self.covariance = self.filter.filter_update(
                filtered_state_mean=self.filt_state_mean,
                filtered_state_covariance=self.covariance,
                observation=observation,
                transition_matrix=transition_matrix,
                observation_matrix=observation_matrix)

        return self.filt_state_mean[0], self.filt_state_mean[1]  # x, y

    def geo_dist(self, latitude, longitude):
        '''assuming the latitude and lontitude are given in degrees'''
        dLat = latitude - self.origin[0]
        dLong = longitude - self.origin[1]
        if dLat == 0:
            angle = 0
        else:
            angle = np.arctan(dLong / dLat)
        radius = 6378.137  # radius of Earth in km
        dLat *= math.pi / 180
        dLong *= math.pi / 180
        a = (math.sin(dLat / 2) * math.sin(dLat / 2) +
             math.sin(dLong / 2) * math.sin(dLong / 2) * math.cos(self.origin[0]) * math.cos(
                     self.origin[1]))
        c = 2 * np.arctan2(math.sqrt(a), math.sqrt(1 - a))
        dist = radius * c * 1000
        dx = dist * math.cos(angle + self.displacement_angle)
        dy = dist * math.sin(angle + self.displacement_angle)
        return dist, dx, dy
