import math

import numpy as np
import pykalman


class KalmanFilter:
    """
    Takes in 6 observations: x, y, change_dist, ax, ay, and heading

    Then converts change_dist and heading into dx and dy
    Then adds gps_dx and gps_dy

    Keeps track of x, y, vx, vy, ax, and ay
    """

    def __init__(self):
        self.prev_gps_x = 0
        self.prev_gps_y = 0

        self.obs_cov = np.identity(8)
        self.trans_cov = np.identity(6)

        self.filter = pykalman.KalmanFilter(
                observation_covariance = self.obs_cov,
                transition_covariance  = self.trans_cov)

        self.filt_state_mean = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
        self.covariance      = np.identity(6)

    def update(self, gps_x, gps_y, gps_flag, gps_dt,
               accel_x, accel_y, acc_flag, acc_dt,
               change_dist, enc_flag, enc_dt,
               upd_dt, heading):

        # need to convert inputs into needed form
        gps_dx = gps_x - self.prev_gps_x
        gps_dy = gps_y - self.prev_gps_y

        enc_dx = change_dist * math.cos(heading)
        enc_dy = change_dist * math.sin(heading)

        observation = np.array([gps_x, gps_y, gps_dx, gps_dy,
                                enc_dx, enc_dy, accel_x, accel_y])

        observation = np.ma.asarray(observation)

        if not gps_flag:
            observation[0] = np.ma.masked
            observation[1] = np.ma.masked
            observation[2] = np.ma.masked
            observation[3] = np.ma.masked

        else:
            self.prev_gps_x = gps_dx
            self.prev_gps_y = gps_dy

        if not enc_flag:
            observation[4] = np.ma.masked
            observation[5] = np.ma.masked

        if not acc_flag:
            observation[6] = np.ma.masked
            observation[7] = np.ma.masked

        obs_matrix = np.array( \
                [[1, 0,      0,      0, 0, 0],
                 [0, 1,      0,      0, 0, 0],
                 [0, 0, gps_dt,      0, 0, 0],
                 [0, 0,      0, gps_dt, 0, 0],
                 [0, 0, enc_dt,      0, 0, 0],
                 [0, 0,      0, enc_dt, 0, 0],
                 [0, 0,      0,      0, 1, 0],
                 [0, 0,      0,      0, 0, 1]])

        transition_matrix = np.array( \
                [[1, 0, upd_dt,      0,      0,      0],
                 [0, 1,      0, upd_dt,      0,      0],
                 [0, 0,      1,      0, upd_dt,      0],
                 [0, 0,      0,      1,      0, upd_dt],
                 [0, 0,      0,      0,      1,      0],
                 [0, 0,      0,      0,      0,      1]])


        self.filt_state_mean, self.covariance = \
                self.filter.filter_update( \
                    filtered_state_mean = self.filt_state_mean,
                    filtered_state_covariance = self.covariance,
                    observation = observation,
                    transition_matrix = transition_matrix,
                    observation_matrix = obs_matrix)

        return self.filt_state_mean[0:2]
filt = KalmanFilter()
thing = filt.update(0,0,0,0,0,0,0,0,0,0,0,0,0)
print( thing)
