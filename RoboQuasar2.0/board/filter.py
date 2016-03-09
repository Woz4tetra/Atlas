"""
Written by Jason Kagie (modified by Ben Warwick)

interpreter.py, written for RoboQuasar2.0
Version 1/4/2016
=========

Interprets sensor data from board/data.py to x, y, theta (current position).

Classes:
    MainFilter - contains a kalman filter tailored for gps, encoder,
        accelerometer and orientation input
"""

import numpy as np
import pykalman
import time
import math


class StateFilter(object):
    def __init__(self):
        self.headingKalman = HeadingKalman()
        self.placeKalman = PositionKalman()

    def update(self, gps_x, gps_y, change_dist, accel_x, accel_y, gyro_z, heading, dt):
        phi = self.headingKalman.update(heading, gyro_z, dt)
        x, y = self.placeKalman.update(gps_x, gps_y, change_dist,
                                       accel_x, accel_y, phi, dt)
        return x, y, phi


class HeadingKalman(object):
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


class PositionKalman(object):
    def __init__(self):
        self.filter = pykalman.KalmanFilter()
        self.filt_state_mean = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.covariance = np.identity(6)

    def update(self, gps_x, gps_y, change_dist, accel_x, accel_y, heading, dt):
        observation = np.array([gps_x, gps_y, change_dist, accel_x, accel_y])

        if (math.cos(heading) == 0):
            y_coeff = dt / math.sin(heading)
            x_coeff = 0

        elif (math.sin(heading) == 0):
            x_coeff = dt / math.cos(heading)
            y_coeff = 0

        else:
            x_coeff = 0.5 * dt / math.cos(heading)
            y_coeff = 0.5 * dt / math.sin(heading)

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
