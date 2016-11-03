import numpy as np
from numpy import linalg

np.set_printoptions(precision=4)

earth_radius = 6378137  # WGS84 Equatorial radius in meters
earth_rotation_rate = 7.292115E-5  # rad/s
eccentricity = 0.0818191908425  # WGS84 eccentricity
earth_gravity_constant = 3.986004418E14
J_2 = 1.082627E-3
speed_of_light = 299792458


class GrovesKalmanFilter:
    def __init__(self, initial_lat, initial_long, initial_alt):
        # convert initials to ECEF
        initial_gps = self.NED_to_ECEF(initial_lat, initial_long, initial_alt)

        # initialize state transition (P_matrix): Initialize_LC_P_matrix
        # P_matrix_propagated in the matlab code
        self.error_covariance_propagated_P = np.matrix(np.eye(15))

        # est_IMU_bias & est_IMU_bias_old in the matlab code
        self.estimated_imu_bias = np.matrix(np.zeros((1, 6)))

        # initialize IMU quantization residuals
        self.quantization_residuals = np.matrix(np.zeros((1, 6)))

        # epoch variable initialization
        self.state_transition_Phi = np.matrix(np.eye(15))  # phi in the paper
        self.error_covariance_P = np.matrix(np.eye(15))

        self.measurement_H = np.matrix(
            np.zeros((6, 15)))  # H_matrix in the matlab code
        self.noise_covariance_R = np.matrix(
            np.zeros((6, 6)))  # R_matrix in the matlab code
        self.kalman_gain_K = np.zeros(
            (6, 6))  # TODO: figure out the dimension of this
        self.kalman_gain_K = np.zeros(
            (6, 6))  # TODO: figure out the dimension of this

        self.measurement_innovation_Z = np.matrix(np.zeros((6, 1)))

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

        self.approx_noise_covariance_Q = np.matrix(np.zeros((15, 15)))
        self.estimated_state_prop_x = np.matrix(
            np.zeros((1, 15)))  # x_est_propagated in the matlab code

        self.prev_t = 0

    # ---- General KF functions -----

    def update_imu(self, t, ax, ay, az, gx, gy, gz):
        # check if the reference frame for acceleration is correct

        # make sure ax, ay, az are specific force

        # convert specific force to ECEF (Kinematics_ECEF), uses initial orientation to generate transformation matrix

        # predict imu errors (IMU_model)
        # correct IMU errors

        # take imu measurements and apply to the INS (Nav_equations_ECEF)
        pass

    def update_gps(self, t, longitude, latitude, altitude):
        # append to GPS data, calculate velocity

        self.filter_epoch()

        # convert back to NED

    # ----- General helper functions -----
    def ECEF_to_NED(self, x, y, z):
        """
        Convert earth center earth fixed coordinates in meters to
        north east down

        :return: latitude (degrees), longitude (degrees), altitude (meters)
        """
        # return latitude, longitude, altitude

    def NED_to_ECEF(self, latitude, longitude, altitude):
        """
        Convert north east down to earth center earth fixed coordinates

        :return: numpy matrix: [x, y, z] (meters)
        """
        # return np.matrix([x, y, z])

    # ---- Epoch functions -----

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
            [0, 0, earth_rotation_rate])  # Omega_ie in the matlab code

        self.state_transition_Phi[0:3, 0:3] -= \
            self.skew_sym_earth_rotation_rate * dt

        self.state_transition_Phi[0:3, 12:15] = self.ins_frame_transformer * dt

        # TODO: get real values for self.ins_frame_transformer
        self.state_transition_Phi[3:6, 0:3] = -dt * self.skew_symmetric(
            self.ins_frame_transformer * specific_force_vector)

        self.state_transition_Phi[3:6, 3:6] -= \
            2 * self.skew_sym_earth_rotation_rate * dt

        geocentric_radius = \
            earth_radius / np.sqrt(
                1 - (eccentricity * np.sin(self.prev_lat)) ** 2) * np.sqrt(
                np.cos(self.prev_lat) ** 2 + (
                    1 - eccentricity ** 2) ** 2 * np.sin(
                    self.prev_lat) ** 2)

        self.state_transition_Phi[3:6, 6:9] = \
            -dt * 2 * self.gravity_ecef(
                self.estimated_position) / geocentric_radius * \
            self.estimated_position.T / np.sqrt(
                self.estimated_position.T * self.estimated_position)

        self.state_transition_Phi[3:6, 9:12] = self.estimated_position * dt

        self.state_transition_Phi[6:9, 3:6] = np.matrix(np.eye(3)) * dt

    def determine_noise_covariance(self, dt):
        """
        Step 2 in the matlab code

        :param dt:
        :return:
        """
        self.approx_noise_covariance_Q = np.matrix(
            np.eye(15))  # Q_prime_matrix in the matlab code

        self.approx_noise_covariance_Q[0:3, 0:3] *= self.gyro_noise_PSD
        self.approx_noise_covariance_Q[3:6, 3:6] *= self.accel_noise_PSD
        self.approx_noise_covariance_Q[9:12, 9:12] *= self.accel_bias_PSD
        self.approx_noise_covariance_Q[12:15, 12:15] *= self.gyro_bias_PSD

        self.approx_noise_covariance_Q *= dt

    def set_propagated_state_est(self):
        """
        Step 3 in the matlab code
        :return:
        """
        self.estimated_state_prop_x = np.matrix(
            np.zeros((1, 15)))  # x_est_propagated in the matlab code

    def set_propagated_error_est(self):
        """
        Step 4 in the matlab code
        :return:
        """
        # P_matrix_propagated in the matlab code
        self.error_covariance_propagated_P = self.state_transition_Phi * (
            self.error_covariance_P + 0.5 * self.approx_noise_covariance_Q) * \
                                             self.state_transition_Phi.T + \
                                    0.5 * self.approx_noise_covariance_Q

    def setup_measurement_matrix(self):
        """
        Step 5 in the matlab code
        :return:
        """
        # H_matrix in the matlab code
        self.measurement_H = np.matrix(np.zeros((6, 15)))
        self.measurement_H[0:3, 6:9] = -np.matrix(np.eye(3))
        self.measurement_H[3:6, 3:6] = -np.matrix(np.eye(3))

    def setup_noise_covariance_matrix(self):
        """
        Step 6 in the matlab code
        :return:
        """
        # R_matrix in the matlab code
        self.noise_covariance_R[0:3, 0:3] = np.matrix(
            np.eye(3)) * self.pos_meas_SD ** 2
        self.noise_covariance_R[0:3, 3:6] = np.matrix(np.zeros((3, 3)))
        self.noise_covariance_R[3:6, 0:3] = np.matrix(np.zeros((3, 3)))
        self.noise_covariance_R[3:6, 0:3] = np.matrix(
            np.eye(3)) * self.vel_meas_SD ** 2

    def calc_kalman_gain(self):
        """
        Step 7 in the matlab code
        :return:
        """
        # K_matrix in the matlab code
        self.kalman_gain_K = self.error_covariance_propagated_P * \
                             self.noise_covariance_R.T * linalg.inv(
            self.measurement_H * self.error_covariance_propagated_P *
            self.noise_covariance_R.T * self.noise_covariance_R)

    def formulate_measurement_innovations(self, gps_pos_ecef, gps_vel_ecef):
        """
        Step 8 in the matlab code

        :param: gps_pos_ecef = [x, y, z] in ECEF frame
        :return:
        """
        # delta_z in the matlab code
        self.measurement_innovation_Z[0:3] = \
            gps_pos_ecef - self.estimated_position
        self.measurement_innovation_Z[3:6] = \
            gps_vel_ecef - self.estimated_velocity

    def update_state_estimates(self):
        """
        Step 9 in the matlab code

        :return:
        """
        # x_est_propagated in the matlab code
        self.estimated_state_prop_x + self.kalman_gain_K * \
                                      self.measurement_innovation_Z

    def update_covariance_matrix(self):
        """
        Step 10 in the matlab code

        :return:
        """
        # P_matrix_new, P_matrix in the matlab code
        self.error_covariance_P = (np.matrix(
            np.eye(15)) - self.kalman_gain_K * self.measurement_H) * \
                                  self.error_covariance_propagated_P

    def correct_estimates(self):
        """
        CLOSED-LOOP CORRECTION in the matlab code
        :return:
        """
        self.ins_frame_transformer = (np.matrix(
            np.eye(3)) - self.skew_symmetric(
            self.estimated_state_prop_x[0:3])) * self.ins_frame_transformer
        self.estimated_attiude *= (np.matrix(np.eye(3)) - self.skew_symmetric(self.estimated_state_prop_x[0:3]))
        self.estimated_velocity -= self.estimated_state_prop_x[3:6]
        self.estimated_position -= self.estimated_state_prop_x[6:9]
        self.estimated_imu_bias -= self.estimated_state_prop_x[9:15]

    # ----- Epoch helper functions -----

    def gravity_ecef(self, estimated_position):
        """
        Inputs:
          pos_ECEF  Cartesian position of body frame w.r.t. ECEF frame, resolved
                  about ECEF-frame axes (m)
        Outputs:
          g       Acceleration due to gravity (m/s^2)
        """
        # Calculate distance from center of the Earth
        mag_r = np.sqrt(estimated_position * (estimated_position.T))

        # If the input position is 0,0,0, produce a dummy output
        if mag_r == 0:
            return np.array([0, 0, 0])

        # Calculates gravitational acceleration
        else:
            z_scale = 5 * (estimated_position[2] / mag_r) ** 2

            matrix = np.matrix([(1 - z_scale) * estimated_position[0],
                                (1 - z_scale) * estimated_position[1],
                                (1 - z_scale) * estimated_position[2]])
            gamma = (-earth_gravity_constant / mag_r ** 3 *
                     (estimated_position + 1.5 * J_2 * (
                         earth_radius / mag_r) ** 2 * matrix))
            # Add centripetal acceleration
            g = [gamma[0] + earth_rotation_rate ** 2 *
                 estimated_position[0],
                 gamma[1] + earth_rotation_rate ** 2 *
                 estimated_position[1],
                 gamma[2]]
            return np.array(g)

    @staticmethod
    def skew_symmetric(m):
        """
        creates a 3v3 skew_symmetric matrix from a 1x3 matrix
        """
        return np.matrix([[0, -m[2], m[2]],
                          [m[2], 0, -m[0]],
                          [-m[1], m[0], 0]])
