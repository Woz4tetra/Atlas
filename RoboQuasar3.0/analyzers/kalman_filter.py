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
        self.covariance = np.array([[1.0,0.0],[0.0,1.0]])
        self.obs_matrix = np.identity(2)
        #observation matrices are how you weight the inputs
        self.observation_covariance = np.array([[1.0,0.0],[0.0,15.0]])

    def update(self, heading, gyro_z, dt):
        trans_matrix = np.array([[1, dt],
                                 [0, 1]])
        obs = np.array([heading, gyro_z])
        self.filt_state_mean, self.covariance = self.filter.filter_update(
            filtered_state_mean=self.filt_state_mean,
            filtered_state_covariance=self.covariance,
            observation=obs,
            transition_matrix=trans_matrix,
            observation_matrix=self.obs_matrix,
            observation_covariance = self.observation_covariance)


        #if the inputs are out of its bounds,
        #reinitialize it to that bound so that it doesnt mess up
        if self.filt_state_mean[0] < 0:
            self.filt_state_mean[0] = 0
            self.filter = pykalman.KalmanFilter(
                    initial_state_mean = self.filt_state_mean,
                    initial_state_covariance = self.covariance)
        if self.filt_state_mean[0] >360:
            self.filt_state_mean[0] = 360
            self.filter = pykalman.KalmanFilter(
                    initial_state_mean = self.filt_state_mean,
                    initial_state_covariance = self.covariance)

        return self.filt_state_mean[0]


class PositionKalman(object):
    """
    Takes in 5 observations: x, y, dist_traveled, ax, ay
    Keeps track of x, y, vx, vy, ax, and ay
    """
    def __init__(self):
        self.obs_covariance = np.array([[1.0, 0.0, 0.0, 0.0, 0.0],
                                        [0.0, 1.0, 0.0, 0.0, 0.0],
                                        [0.0, 0.0, 1.0, 0.0, 0.0],
                                        [0.0, 0.0, 0.0, 1.0, 0.0],
                                        [0.0, 0.0, 0.0, 0.0, 1.0]])
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
