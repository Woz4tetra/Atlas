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

    def update(self, gps_x, gps_y, change_dist, enc_flag,
            accel_x, accel_y, heading, dt, acc_flag, gps_flag):
        data = self.placeKalman.update(gps_x, gps_y, change_dist, enc_flag,
                                       accel_x, accel_y, heading,
                                       dt, acc_flag, gps_flag)
        return data


class PositionKalman(object):
    """
    Takes in 6 observations: x, y, dx, dy, ax, ay
    Keeps track of x, y, vx, vy, ax, and ay
    """
    def __init__(self):
        self.obs_covariance = \
            np.array([[1, 0, 0, 0, 0, 0],
                      [0, 1, 0, 0, 0, 0],
                      [0, 0, 1, 0, 0, 0],
                      [0, 0, 0, 1, 0, 0],
                      [0, 0, 0, 0, 1, 0],
                      [0, 0, 0, 0, 0, 1]])

#TODO make the velocity/encoder not fucking shit

        self.trans_covariance = \
            np.array([[1, 0, 0, 0, 0, 0],
                      [0, 1, 0, 0, 0, 0],
                      [0, 0, 100, 0, 0, 0],
                      [0, 0, 0, 100, 0, 0],
                      [0, 0, 0, 0, 1, 0],
                      [0, 0, 0, 0, 0, 1]])

        self.filter = pykalman.KalmanFilter(
                observation_covariance = self.obs_covariance,
                transition_covariance  = self.trans_covariance)
        self.filt_state_mean = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.covariance = np.identity(6)
        self.count = 0

    def update(self, gps_x, gps_y, change_dist, enc_flag,
            accel_x, accel_y, heading, dt, acc_flag, gps_flag):

        #need to convert change_dist into dx,dy
        dx = change_dist * math.cos(heading)
        dy = change_dist * math.sin(heading)

        observation = np.array([gps_x, gps_y,
            dx, dy, accel_x, accel_y])

        observation = np.ma.asarray(observation)

        self.count +=1

        if self.count > 1:
            observation[0] = np.ma.masked
            observation[1] = np.ma.masked

        if self.count == 35:
            self.count = 0

        if gps_flag:
            observation[0] = np.ma.masked
            observation[1] = np.ma.masked

        if enc_flag:
            observation[2] = np.ma.masked
            observation[3] = np.ma.masked

        if acc_flag:
            observation[4] = np.ma.masked
            observation[5] = np.ma.masked

        observation_matrix = np.array(
            [[1, 0, 0,  0, 0, 0],
             [0, 1, 0,  0, 0, 0],
             [0, 0, dt, 0, 0, 0],
             [0, 0, 0, dt, 0, 0],
             [0, 0, 0, 0,  1, 0],
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

        return self.filt_state_mean
    # all of them
