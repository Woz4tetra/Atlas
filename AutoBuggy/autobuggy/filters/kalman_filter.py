import numpy as np


class KalmanFilter:
    def __init__(self):
        self.earth_radius = 6378137  # WGS84 Equatorial radius in meters
        self.speed_of_light = 299792458  # m/s
        self.earth_rotation_rate = 7.292115E-5  # rad/s
        self.eccentricity = 0.0818191908425  # WGS84 eccentricity

        self.state_transition = np.matrix(np.eye(15))  # phi in the paper

        self.ins_frame_transformer = np.matrix(
            [0, 0, 0])  # C_b_e or est_C_b_e_old in the paper, imu body to ECEF
        self.prev_lat = 0  # latitude of previous measurement, est_L_b_old in the matlab code

        self.gyro_noise_PSD = 0
        self.accel_noise_PSD = 0
        self.accel_bias_PSD = 0
        self.gyro_bias_PSD = 0
        self.pos_meas_SD = 0
        self.vel_meas_SD = 0

        # estimated position in the ECEF frame, est_r_eb_e_old in the matlab code
        self.estimated_position = np.matrix([0, 0, 0])
        self.system_noise_covariance = np.matrix(np.zeros((15, 15)))
        self.propagated_state_est = np.matrix(
            np.zeros((1, 15)))  # x_est_propagated in the matlab code
        # self.propagated_noise_est =

        np.set_printoptions(precision=4)

    def update_imu(self, t, ax, ay, az):
        # check if the reference frame for acceleration is correct
        pass

    def update_gps(self, t, longitude, latitude, altitude):
        pass

    def filter_epoch(self, dt, specific_force_vector):
        self.determine_transition_matrix(dt, specific_force_vector)
        self.determine_noise_covariance(dt)
        self.set_propagated_state_est()

    def determine_transition_matrix(self, dt, specific_force_vector):
        """
        Find state_transition matrix (Phi_matrix in the matlab code)
        Step 1 in the matlab code

        :param dt:
        :param specific_force_vector:
        :return:
        """

        # Skew symmetric matrix of Earth rate
        self.skew_sym_earth_rotation_rate = self.skew_symmetric(
            [0, 0, self.earth_rotation_rate])  # Omega_ie in the matlab code

        self.state_transition[0:3, 0:3] -= \
            self.skew_sym_earth_rotation_rate * dt

        self.state_transition[0:3, 12:15] = self.ins_frame_transformer * dt

        # TODO: get real values for self.ins_frame_transformer
        self.state_transition[3:6, 0:3] = -dt * self.skew_symmetric(
            self.ins_frame_transformer * specific_force_vector)

        self.state_transition[3:6, 3:6] -= \
            2 * self.skew_sym_earth_rotation_rate * dt

        geocentric_radius = \
            self.earth_radius / np.sqrt(
                1 - (self.eccentricity * np.sin(self.prev_lat)) ** 2) * np.sqrt(
                np.cos(self.prev_lat) ** 2 + (
                    1 - self.eccentricity ** 2) ** 2 * np.sin(
                    self.prev_lat) ** 2)

        self.state_transition[3:6, 6:9] = \
            -dt * 2 * self.gravity_ecef(
                self.estimated_position) / geocentric_radius * \
            self.estimated_position.T / np.sqrt(
                self.estimated_position.T * self.estimated_position)

        self.state_transition[3:6, 9:12] = self.estimated_position * dt

        self.state_transition[6:9, 3:6] = np.matrix(np.eye(3)) * dt

    def determine_noise_covariance(self, dt):
        """

        Step 2 in the matlab code

        :param dt:
        :return:
        """
        self.system_noise_covariance = np.matrix(np.eye(15))

        self.system_noise_covariance[0:3, 0:3] *= self.gyro_noise_PSD
        self.system_noise_covariance[3:6, 3:6] *= self.accel_noise_PSD
        self.system_noise_covariance[9:12, 9:12] *= self.accel_bias_PSD
        self.system_noise_covariance[12:15, 12:15] *= self.gyro_bias_PSD

        self.system_noise_covariance *= dt

    def set_propagated_state_est(self):
        """
        Step 3 in the matlab code
        :return:
        """
        self.propagated_state_est = np.matrix(np.zeros((1, 15)))

    # ----- Helper functions -----

    def skew_symmetric(self, m):
        """
        creates a 3v3 skew_symmetric matrix from a 1x3 matrix
        """
        return np.matrix([[0, -m[2], m[2]],
                          [m[2], 0, -m[0]],
                          [-m[1], m[0], 0]])

    def gravity_ecef(self, estimated_position):
        return 9.81
