import math

import numpy as np
import pykalman

class HeadingFilter:
    """
    Takes in 3 observationss:
        gps_heading, imu_heading, bind_heading

    imu_heading is relative, so will be used as a differential

    gps and bind headings are absolute

    Keepes track of heading and delta heading
    """

    def __init__(self):
        self.prev_imu = 0

        self.obs_cov = np.identity(3)
        self.trans_cov = np.identity(2)

        self.filter = pykalman.KalmanFilter(
                observation_covariance = self.obs_cov,
                transition_covariance  = self.trans_cov
        )

        self.filt_state_mean = np.array([0.0,0.0])
        self.covariance = np.identity(2)

    def update(self, gps_heading, gps_flag,
               bind_heading, bind_flag,
               imu_heading, imu_flag):

        delta_heading = imu_heading - self.prev_imu

        observation = np.array([gps_heading, bind_heading, delta_heading])

        observation = np.ma.asarray(observation)

        if not gps_flag:
            observation[0] = np.ma.masked

        if not bind_flag:
            observation[1] = np.ma.masked

        if not imu_flag:
            observation[2] = np.ma.masked

        else:
            self.prev_imu = imu_heading

        obs_matrix = np.array(
            [[1,0],
             [1,0],
             [0,1]]
        )

        trans_matrix = np.array(
            [[1,1],
             [0,1]]
        )

        self.filt_state_mean, self.covariance = self.filter.filter_update(
                    filtered_state_mean = self.filt_state_mean,
                    filtered_state_covariance = self.covariance,
                    observation = observation,
                    transition_matrix = trans_matrix,
                    observation_matrix = obs_matrix
        )

        return self.filt_state_mean[0]

