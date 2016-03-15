"""
Written by Jason Kagie (modified by Ben Warwick)

kalman_filter.py, written for RoboQuasar3.0
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
        self.placeKalman = PositionKalman()

    def update(self, gps_x, gps_y, change_dist, accel_x, accel_y, heading, dt):
        #changed so that it no longer runs a filter on the yaw value
        x, y = self.placeKalman.update(gps_x, gps_y, change_dist,
                                       accel_x, accel_y, heading, dt)
        return x, y, heading


class PositionKalman(object):
    """
    Takes in 5 observations: x, y, dist_traveled, ax, ay
    Keeps track of x, y, vx, vy, ax, and ay
    """
    def __init__(self):
        self.obs_covariance = np.array([[1.0, 0.0, 0.0, 0.0, 0.0],
                                        [0.0, 1.0, 0.0, 0.0, 0.0],
                                        [0.0, 0.0, 100.0, 0.0, 0.0],
                                        [0.0, 0.0, 0.0, 100.0, 0.0],
                                        [0.0, 0.0, 0.0, 0.0, 100.0]])
        self.filter = pykalman.KalmanFilter(
                observation_covariance = self.obs_covariance)
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

        return self.filt_state_mean[0], self.filt_state_mean[1]
    # x, y
