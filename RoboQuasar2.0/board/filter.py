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
    def __init__(self, latitude, longitude, circum, heading,
                 prev_encoder_value):
        self.headingKalman = HeadingKalman()
        self.placeKalman = PositionKalman((latitude, longitude), circum,
                                          heading, prev_encoder_value)
        self.time = time.time()
        self.dt = 0

    def update(self, gps_lat, gps_lon, encoder_counts, accel_x, accel_y, gyro_z,
               heading):
        self.dt = 0.105
        self.time = time.time()
        phi = self.headingKalman.update(heading, gyro_z, self.dt)
        x, y = self.placeKalman.update(gps_lat, gps_lon, encoder_counts,
                                       accel_x, accel_y, phi, self.dt)
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
    def __init__(self, origin, wheel_radius, start_heading, prev_encoder_data):
        self.origin = (
            origin[0] / 3600,
            origin[1] / 3600
        )

        self.origin_rad = (
            self.origin[0] * math.pi / 180,
            self.origin[1] * math.pi / 180
        )

        self.wheel_radius = 2 * math.pi * wheel_radius
        self.filter = pykalman.KalmanFilter()
        self.filt_state_mean = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.covariance = np.identity(6)
        self.displacement_angle = start_heading
        self.earth_radius = 6371000
        self.prev_encoder = prev_encoder_data
        self.seconds_conversion = 1850.0 / 60.0  # 1 sec = this many meters

    def update(self, gps_lat, gps_lon, encoder_value, accel_x, accel_y, heading,
               dt):
        x_gps, y_gps = self.geo_dist(gps_lat, gps_lon)

        encoder_counts = encoder_value - self.prev_encoder
        self.prev_encoder = encoder_value

        observation = np.array([x_gps, y_gps,
                                encoder_counts, accel_x, accel_y])

        if (math.cos(heading) == 0):
            y_coeff = self.wheel_radius * dt / math.sin(heading)
            x_coeff = 0

        elif (math.sin(heading) == 0):
            x_coeff = self.wheel_radius * dt / math.cos(heading)
            y_coeff = 0

        else:
            x_coeff = 0.5 * self.wheel_radius * dt / math.cos(heading)
            y_coeff = 0.5 * self.wheel_radius * dt / math.sin(heading)

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
        """assuming the latitude and lontitude are given in seconds"""
        phi1 = self.origin_rad[0]
        phi2 = latitude / 3600 * math.pi / 180
        dphi = phi2 - self.origin_rad[0]
        dlambda = longitude / 3600 * math.pi / 180 - self.origin_rad[1]

        a = (math.sin(dphi / 2) ** 2 +
             math.cos(phi1) * math.cos(phi2) *
             math.sin(dlambda / 2) ** 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        dist = self.earth_radius * c
        angle = math.atan2(latitude - self.origin[0],
                           longitude - self.origin[1])
        return dist * math.cos(angle), dist * math.sin(angle)
