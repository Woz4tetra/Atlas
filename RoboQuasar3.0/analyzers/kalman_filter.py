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

import math

import numpy as np
import pykalman


class PositionFilter:
    """
    Takes in 4 observations: x, y, change_dist, and heading

    Then converts change_dist and heading into dx and dy

    Inputs: gps_x, gps_y, enc_dx, enc_dy
    Keeps track of x, y, vx, vy
    """

    def __init__(self):
        self.obs_cov = [[40, 0, 0, 0],  # gps_x
                        [0, 40, 0, 0],  # gps_y
                        [0, 0, 1, 0],  # enc_dx
                        [0, 0, 0, 1]]  # enc_dy
        self.trans_cov = [[.1, 0, 0, 0],  # x
                          [0, .1, 0, 0],  # y
                          [0, 0, 10, 0],  # vx
                          [0, 0, 0, 10]]  # vy

        self.filter = pykalman.KalmanFilter(
            observation_covariance=self.obs_cov,
            transition_covariance=self.trans_cov)

        self.filt_state_mean = np.array([0.0, 0.0, 0.0, 0.0])
        self.covariance = np.identity(4)

    def update(self, gps_x, gps_y, change_dist, upd_dt, heading):

        # need to convert inputs into needed form

        enc_dx = change_dist * math.cos(heading)
        enc_dy = change_dist * math.sin(heading)

        observation = np.array([gps_x, gps_y, enc_dx, enc_dy])

        obs_matrix = np.array(
            [[1, 0, 0, 0],
             [0, 1, 0, 0],
             [0, 0, upd_dt, 0],
             [0, 0, 0, upd_dt]])

        transition_matrix = np.array(
            [[1, 0, upd_dt, 0],
             [0, 1, 0, upd_dt],
             [0, 0, 1, 0],
             [0, 0, 0, 1]])

        self.filt_state_mean, self.covariance = \
            self.filter.filter_update(
                filtered_state_mean=self.filt_state_mean,
                filtered_state_covariance=self.covariance,
                observation=observation,
                transition_matrix=transition_matrix,
                observation_matrix=obs_matrix)

        return self.filt_state_mean[0:2]


class HeadingFilter:
    """
    Takes in 3 observations:
        gps_heading, imu_heading, bind_heading

    imu_heading is relative, so will be used as a differential

    gps and bind headings are absolute

    Keeps track of heading and delta heading
    """

    def __init__(self):
        self.prev_imu = 0.0
        self.obs_cov = [[1000, 0, 0],
                        [0, 1, 0],
                        [0, 0, 10]]
        self.trans_cov = [[10, 0],
                          [0, 1]]

        self.filter = pykalman.KalmanFilter(
            observation_covariance=self.obs_cov,
            transition_covariance=self.trans_cov
        )

        self.filt_state_mean = np.array([0.0, 0.0])
        self.covariance = np.identity(2)

    def update(self, gps_heading, bind_heading, imu_heading, dt):

        delta_heading = imu_heading - self.prev_imu
        observation = np.array([gps_heading, bind_heading, delta_heading])

        if imu_heading != self.prev_imu:
            self.prev_imu = imu_heading

        obs_matrix = np.array(
            [[1, 0],
             [1, 0],
             [0, 1/dt]]
        )

        trans_matrix = np.array(
            [[1, dt],
             [0, 1]]
        )

        self.filt_state_mean, self.covariance = self.filter.filter_update(
            filtered_state_mean=self.filt_state_mean,
            filtered_state_covariance=self.covariance,
            observation=observation,
            transition_matrix=trans_matrix,
            observation_matrix=obs_matrix
        )

        return self.filt_state_mean[0]
