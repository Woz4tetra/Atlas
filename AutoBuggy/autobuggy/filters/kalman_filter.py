import numpy as np
from numpy import linalg

np.set_printoptions(precision=4)


class KalmanFilter:
    def __init__(self):
        self.epoch = KalmanFilterEpoch()

        self.prev_t = 0

    def update_imu(self, t, ax, ay, az):
        # check if the reference frame for acceleration is correct
        pass

    def update_gps(self, t, longitude, latitude, altitude):
        self.epoch.filter_epoch()


class KalmanFilterEpoch:
    def __init__(self):
        self.earth_radius = 6378137  # WGS84 Equatorial radius in meters
        self.earth_rotation_rate = 7.292115E-5  # rad/s
        self.eccentricity = 0.0818191908425  # WGS84 eccentricity

        self.state_transition = np.matrix(np.eye(15))  # phi in the paper
        self.prev_state_transition = np.matrix(np.eye(15))

        self.measurement = np.matrix(
            np.zeros((6, 15)))  # H_matrix in the matlab code
        self.noise_covariance_matrix = np.matrix(
            np.zeros((6, 6)))  # R_matrix in the matlab code
        self.kalman_gain = np.zeros(
            (6, 6))  # TODO: figure out the dimension of this
        self.kalman_gain = np.zeros(
            (6, 6))  # TODO: figure out the dimension of this

        self.measurement_innovation = np.matrix(np.zeros((6, 1)))

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
        self.estimated_velocity = np.matrix(
            [0, 0, 0])  # est_v_eb_e_old in the matlab code
        self.estimated_imu_bias = np.matrix(
            [0, 0, 0, 0, 0, 0])  # est_IMU_bias_old in the matlab code

        self.system_noise_covariance = np.matrix(np.zeros((15, 15)))
        self.propagated_state_est = np.matrix(
            np.zeros((1, 15)))  # x_est_propagated in the matlab code
        self.propagated_noise_est = np.matrix(
            np.eye(15))  # P_matrix_propagated in the matlab code

    def filter_epoch(self, dt, specific_force_vector, gps_pos_ecef,
                     gps_vel_ecef):
        self.determine_transition_matrix(dt, specific_force_vector)
        self.determine_noise_covariance(dt)
        self.set_propagated_state_est()
        self.set_propagated_error_est()
        self.setup_measurement_matrix()
        self.setup_noise_covariance_matrix()
        self.calc_kalman_gain()
        self.formulate_measurement_innovations(gps_pos_ecef, gps_vel_ecef)
        self.update_state_estimates()
        self.update_covariance_matrix()
        self.correct_estimates()

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
        self.system_noise_covariance = np.matrix(
            np.eye(15))  # Q_prime_matrix in the matlab code

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
        self.propagated_state_est = np.matrix(
            np.zeros((1, 15)))  # x_est_propagated in the matlab code

    def set_propagated_error_est(self):
        """
        Step 4 in the matlab code
        :return:
        """
        # P_matrix_propagated in the matlab code
        self.propagated_noise_est = self.state_transition * (
            self.prev_state_transition + 0.5 * self.system_noise_covariance) * \
                                    self.state_transition.T + \
                                    0.5 * self.system_noise_covariance

    def setup_measurement_matrix(self):
        """
        Step 5 in the matlab code
        :return:
        """
        # H_matrix in the matlab code
        self.measurement = np.matrix(np.zeros((6, 15)))
        self.measurement[0:3, 6:9] = -np.matrix(np.eye(3))
        self.measurement[3:6, 3:6] = -np.matrix(np.eye(3))

    def setup_noise_covariance_matrix(self):
        """
        Step 6 in the matlab code
        :return:
        """
        # R_matrix in the matlab code
        self.noise_covariance_matrix[0:3, 0:3] = np.matrix(
            np.eye(3)) * self.pos_meas_SD ** 2
        self.noise_covariance_matrix[0:3, 3:6] = np.matrix(np.zeros((3, 3)))
        self.noise_covariance_matrix[3:6, 0:3] = np.matrix(np.zeros((3, 3)))
        self.noise_covariance_matrix[3:6, 0:3] = np.matrix(
            np.eye(3)) * self.vel_meas_SD ** 2

    def calc_kalman_gain(self):
        """
        Step 7 in the matlab code
        :return:
        """
        # K_matrix in the matlab code
        self.kalman_gain = self.propagated_noise_est * \
                           self.noise_covariance_matrix.T * linalg.inv(
            self.measurement * self.propagated_noise_est *
            self.noise_covariance_matrix.T * self.noise_covariance_matrix)

    def formulate_measurement_innovations(self, gps_pos_ecef, gps_vel_ecef):
        """
        Step 8 in the matlab code

        :param: gps_pos_ecef = [x, y, z] in ECEF frame
        :return:
        """
        # delta_z in the matlab code
        self.measurement_innovation[0:3] = \
            gps_pos_ecef - self.estimated_position
        self.measurement_innovation[3:6] = \
            gps_vel_ecef - self.estimated_velocity

    def update_state_estimates(self):
        """
        Step 9 in the matlab code

        :return:
        """
        # x_est_propagated in the matlab code
        self.propagated_state_est += self.kalman_gain * \
                                     self.measurement_innovation

    def update_covariance_matrix(self):
        """
        Step 10 in the matlab code

        :return:
        """
        # P_matrix_new, P_matrix in the matlab code
        self.state_transition = (np.matrix(
            np.eye(15)) - self.kalman_gain * self.measurement) * \
                                self.propagated_noise_est

    def correct_estimates(self):
        """
        CLOSED-LOOP CORRECTION in the matlab code
        :return:
        """
        self.ins_frame_transformer = (np.matrix(
            np.eye(3)) - self.skew_symmetric(
            self.propagated_state_est[0:3])) * self.ins_frame_transformer
        self.estimated_velocity -= self.propagated_state_est[3:6]
        self.estimated_position -= self.propagated_state_est[6:9]
        self.estimated_imu_bias -= self.propagated_state_est[9:15]

    # ----- Helper functions -----

    @staticmethod
    def skew_symmetric(m):
        """
        creates a 3v3 skew_symmetric matrix from a 1x3 matrix
        """
        return np.matrix([[0, -m[2], m[2]],
                          [m[2], 0, -m[0]],
                          [-m[1], m[0], 0]])

    def gravity_ecef(self, estimated_position):
        return 9.81
