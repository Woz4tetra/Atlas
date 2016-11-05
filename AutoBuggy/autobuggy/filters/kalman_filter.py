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
    def __init__(self, initial_roll_ecef, initial_pitch_ecef, initial_yaw_ecef,
                 roll_error, pitch_error, yaw_error,
                 initial_lat, initial_long, initial_alt,
                 initial_v_north, initial_v_east, initial_v_down,

                 initial_attitude_unc, initial_velocity_unc,
                 initial_position_unc, initial_accel_bias_unc,
                 initial_gyro_bias_unc, initial_clock_offset_unc,
                 initial_clock_drift_unc):
        # initialize properties
        self.initial_roll_ecef = initial_roll_ecef
        self.initial_pitch_ecef = initial_pitch_ecef
        self.initial_yaw_ecef, = initial_yaw_ecef,
        self.initial_roll_error = roll_error
        self.initial_pitch_error = pitch_error
        self.initial_yaw_error, = yaw_error,
        self.initial_lat = initial_lat
        self.initial_long = initial_long
        self.initial_alt, = initial_alt,
        self.initial_v_north = initial_v_north
        self.initial_v_east = initial_v_east
        self.initial_v_down = initial_v_down

        # old_est_C_b_e in Loosely_coupled_INS_GNSS
        self.estimated_body_to_ecef_transform = \
            self.init_body_to_ecef_transform()

        # P_matrix & est_IMU_bias in Loosely_coupled_INS_GNSS
        self.error_covariance_P = \
            self.init_P(initial_attitude_unc, initial_velocity_unc,
                        initial_position_unc, initial_accel_bias_unc,
                        initial_gyro_bias_unc, initial_clock_offset_unc,
                        initial_clock_drift_unc)
        self.estimated_imu_biases = np.matrix(np.zeros((1, 6)))

        # prev gps variables
        self.prev_lat = self.initial_lat
        self.prev_long = self.initial_long
        self.prev_alt = self.initial_alt

        self.ins = INS()
        self.epoch = Epoch()

        # Outputs of the Kalman filter
        self.estimated_position = np.matrix([0, 0, 0])
        self.estimated_velocity = np.matrix([0, 0, 0])
        self.estimated_attitude = np.matrix([0, 0, 0])

    def init_P(self, initial_attitude_unc, initial_velocity_unc,
               initial_position_unc, initial_accel_bias_unc,
               initial_gyro_bias_unc, initial_clock_offset_unc,
               initial_clock_drift_unc):
        error_covariance_P = np.matrix(np.zeros((17, 17)))

        I_3 = np.matrix(np.eye(3))
        error_covariance_P[0:3, 0:3] = I_3 * initial_attitude_unc ** 2
        error_covariance_P[3:6, 3:6] = I_3 * initial_velocity_unc ** 2
        error_covariance_P[6:9, 6:9] = I_3 * initial_position_unc ** 2
        error_covariance_P[9:12, 9:12] = I_3 * initial_accel_bias_unc ** 2
        error_covariance_P[12:15, 12:15] = I_3 * initial_gyro_bias_unc ** 2
        error_covariance_P[15, 15] = I_3 * initial_clock_offset_unc ** 2
        error_covariance_P[16, 16] = I_3 * initial_clock_drift_unc ** 2

        return error_covariance_P

    def init_body_to_ecef_transform(self):
        """
        Initialize estimated attitude solution
        Take the guess of initial orientation and the estimate of error from NED
        and return the transformation from the body frame to ECEF
        """

        # true_C_b_n in Loosely_coupled_INS_GNSS
        initial_attitude = euler_to_ctm(self.initial_roll_ecef,
                                        self.initial_pitch_ecef,
                                        self.initial_yaw_ecef).T

        # delta_C_b_n in Initialize_NED_attitude
        attitude_error = euler_to_ctm(-self.initial_roll_error,
                                      -self.initial_pitch_error,
                                      -self.initial_yaw_error)

        # est_C_b_n in Initialize_NED_attitude
        estimated_attitude = attitude_error * initial_attitude

        # old_est_C_b_e in Loosely_coupled_INS_GNSS
        estimated_body_transform = ned_to_ecef_general(*estimated_attitude)

        return estimated_body_transform

    def get_gps_v(self, gps_dt, lat, long, altitude):
        v_north = (lat - self.prev_lat) / gps_dt
        v_east = (long - self.prev_long) / gps_dt
        v_down = (altitude - self.prev_alt) / gps_dt

        self.prev_lat = lat
        self.prev_long = long
        self.prev_alt = altitude

        return v_north, v_east, v_down

    def imu_updated(self, imu_dt, ax, ay, az, gx, gy, gz,
                    # accel_bias, gyro_bias
                    ):
        self.estimated_position, self.estimated_velocity, \
        self.estimated_attitude = \
            self.ins.update(
                imu_dt, np.matrix([ax, ay, az]), np.matrix([gx, gy, gz]),
                # accel_bias, gyro_bias,
                self.estimated_position, self.estimated_velocity,
                self.estimated_attitude)

    def gps_updated(self, gps_dt, lat, long, altitude):
        # convert lat, long, altitude to position and velocity in ECEF
        v_north, v_east, v_down = self.get_gps_v(gps_dt, lat, long, altitude)
        gps_position_ecef = ned_to_ecef_position(lat, long, altitude)
        gps_velocity_ecef = ned_to_ecef_general(v_north, v_east, v_down)

        self.epoch.update(
            gps_dt, gps_position_ecef, gps_velocity_ecef,
            self.estimated_position, self.estimated_velocity,
            self.estimated_attitude,
            self.estimated_imu_biases,
            self.error_covariance_P,
            self.ins.accel_measurement, self.ins.gyro_measurement
        )

        return self.estimated_position


class Epoch:
    def __init__(self):
        self.skew_sym_earth_rotation_rate = skew_symmetric(
            [0, 0, earth_rotation_rate])  # Omega_ie in the matlab code
        self.state_transition_Phi = None  # Phi_matrix in the matlab code
        self.approx_noise_covariance_Q = None

        self.estimated_position = None
        self.estimated_velocity = None
        self.estimated_body_to_ecef_transform = None
        self.accel_measurement = None
        self.gyro_measurement = None

        self.gps_position_ecef = None
        self.gps_velocity_ecef = None
        self.estimated_imu_biases = None

        self.error_covariance_P = None
        self.error_covariance_propagated_P = None
        self.approx_noise_covariance_Q = None
        self.measurement_H = None
        self.noise_covariance_R = None
        self.kalman_gain_K = None
        self.measurement_innovation_Z = None

    def update(self, dt, gps_position_ecef, gps_velocity_ecef,
               estimated_position, estimated_velocity,
               estimated_body_to_ecef_transform,
               estimated_imu_biases, error_covariance_P, accel_measurement,
               gyro_measurement):
        self.estimated_position = estimated_position
        self.estimated_velocity = estimated_velocity
        self.estimated_body_to_ecef_transform = estimated_body_to_ecef_transform
        self.accel_measurement = accel_measurement
        self.gyro_measurement = gyro_measurement

        self.gps_position_ecef = gps_position_ecef
        self.gps_velocity_ecef = gps_velocity_ecef
        self.estimated_imu_biases = estimated_imu_biases

        self.error_covariance_P = error_covariance_P

        self.determine_transition_matrix(dt)
        self.determine_noise_covariance(dt)

    def determine_transition_matrix(self, dt):
        estimated_lat = ecef_to_ned(*self.estimated_position)[0]

        self.state_transition_Phi = np.matrix(np.eye(15))

        self.state_transition_Phi[0:3, 0:3] -= \
            self.skew_sym_earth_rotation_rate * dt

        self.state_transition_Phi[0:3, 12:15] = \
            self.estimated_body_to_ecef_transform * dt

        self.state_transition_Phi[3:6, 0:3] = -dt * skew_symmetric(
            self.estimated_body_to_ecef_transform * self.accel_measurement)

        self.state_transition_Phi[3:6, 3:6] -= \
            2 * self.skew_sym_earth_rotation_rate * dt

        geocentric_radius = \
            earth_radius / np.sqrt(
                1 - (eccentricity * np.sin(estimated_lat)) ** 2) * np.sqrt(
                np.cos(estimated_lat) ** 2 + (
                    1 - eccentricity ** 2) ** 2 * np.sin(
                    estimated_lat) ** 2)

        self.state_transition_Phi[3:6, 6:9] = \
            -dt * 2 * gravity_ecef(
                *self.estimated_position) / geocentric_radius * \
            self.estimated_position.T / np.sqrt(
                self.estimated_position.T * self.estimated_position)

        self.state_transition_Phi[3:6, 9:12] = self.estimated_position * dt

        self.state_transition_Phi[6:9, 3:6] = np.matrix(np.eye(3)) * dt

    def determine_noise_covariance(self, dt):
        # Q_prime_matrix in the matlab code
        self.approx_noise_covariance_Q = np.matrix(np.eye(15))

        self.approx_noise_covariance_Q[0:3, 0:3] *= self.gyro_noise_PSD
        self.approx_noise_covariance_Q[3:6, 3:6] *= self.accel_noise_PSD
        self.approx_noise_covariance_Q[9:12, 9:12] *= self.accel_bias_PSD
        self.approx_noise_covariance_Q[12:15, 12:15] *= self.gyro_bias_PSD

        self.approx_noise_covariance_Q *= dt

    def set_propagated_state_est(self):
        self.estimated_state_prop_x = np.matrix(np.zeros((1, 15)))
        # x_est_propagated in the matlab code

    def set_propagated_error_est(self):
        self.error_covariance_propagated_P = self.state_transition_Phi * (
            self.error_covariance_P + 0.5 * self.approx_noise_covariance_Q) * \
                                             self.state_transition_Phi.T + \
                                             0.5 * self.approx_noise_covariance_Q

    def setup_noise_covariance_matrix(self):
        # R_matrix in the matlab code
        self.noise_covariance_R = np.matrix(np.eye(6))
        self.noise_covariance_R[0:3, 0:3] = np.matrix(
            np.eye(3)) * self.pos_meas_SD ** 2
        self.noise_covariance_R[0:3, 3:6] = np.matrix(np.zeros((3, 3)))
        self.noise_covariance_R[3:6, 0:3] = np.matrix(np.zeros((3, 3)))
        self.noise_covariance_R[3:6, 0:3] = np.matrix(
            np.eye(3)) * self.vel_meas_SD ** 2

    def calc_kalman_gain(self):
        self.kalman_gain_K = self.error_covariance_propagated_P * \
                             self.noise_covariance_R.T * linalg.inv(
            self.measurement_H * self.error_covariance_propagated_P *
            self.noise_covariance_R.T * self.noise_covariance_R)

    def formulate_measurement_innovations(self):
        self.measurement_innovation_Z[0:3] = \
            self.gps_position_ecef - self.estimated_position
        self.measurement_innovation_Z[3:6] = \
            self.gps_velocity_ecef - self.estimated_velocity

    def setup_measurement_matrix(self):
        # H_matrix in the matlab code
        self.measurement_H = np.matrix(np.zeros((6, 15)))
        self.measurement_H[0:3, 6:9] = -np.matrix(np.eye(3))
        self.measurement_H[3:6, 3:6] = -np.matrix(np.eye(3))

    def update_state_estimates(self):
        # x_est_propagated in the matlab code
        self.estimated_state_prop_x + self.kalman_gain_K * \
                                      self.measurement_innovation_Z

    def update_covariance_matrix(self):
        # P_matrix_new, P_matrix in the matlab code
        self.error_covariance_P = (np.matrix(
            np.eye(15)) - self.kalman_gain_K * self.measurement_H) * \
                                  self.error_covariance_propagated_P

    def correct_estimates(self):
        self.estimated_body_to_ecef_transform *= (np.matrix(
            np.eye(3)) - skew_symmetric(self.estimated_state_prop_x[0:3]))
        self.estimated_velocity -= self.estimated_state_prop_x[3:6]
        self.estimated_position -= self.estimated_state_prop_x[6:9]
        self.estimated_imu_biases -= self.estimated_state_prop_x[9:15]


class INS:
    def __init__(self):
        self.quantization_residuals = np.matrix(np.zeros((6, 1)))

        self.accel_measurement = np.matrix([0, 0, 0])
        self.gyro_measurement = np.matrix([0, 0, 0])

    def update(self, dt, accel_measurement, gyro_measurement,
               # accel_bias_measurement, gyro_bias_measurement,
               estimated_position, estimated_velocity,
               estimated_body_to_ecef_transform,
               # estimated_IMU_biases
               ):
        # (maybe) TODO: IMU_model, estimate IMU noise, assign to accel_measurement and gyro_measurement

        self.accel_measurement = accel_measurement
        self.gyro_measurement = gyro_measurement

        # self.accel_measurement -= estimated_IMU_biases[0:3]
        # self.gyro_measurement -= estimated_IMU_biases[3:6]

        # Nav_equations_ECEF, apply INS to position estimate
        estimated_position, estimated_velocity, estimated_body_to_ecef_transform = \
            self.estimate_ins_state(dt, estimated_position, estimated_velocity,
                                    estimated_body_to_ecef_transform,
                                    accel_measurement, gyro_measurement)

        return estimated_velocity, estimated_velocity, estimated_body_to_ecef_transform

    def estimate_ins_state(self, dt, estimated_position, estimated_velocity,
                           estimated_body_to_ecef_transform, accel_measurement,
                           gyro_measurement):
        # Nav_equations_ECEF function
        pass


def skew_symmetric(m):
    """
    creates a 3v3 skew_symmetric matrix from a 1x3 matrix
    """
    return np.matrix([[0, -m[2], m[2]],
                      [m[2], 0, -m[0]],
                      [-m[1], m[0], 0]])


def euler_to_ctm(roll, pitch, yaw):
    """
    Purpose
    ----------
    Converts a set of Euler angles to the corresponding coordinate transformation matrix

    Parameters
    ----------
    eul: Euler angles describing the rotation from beta to alpha in the order (roll, pitch, yaw)

    Output
    ----------
    C: coordinate transformation matrix describing transformation from beta to alpha

    """

    # precalculate sines and cosines of the Euler angles
    sin_phi = np.sin(roll)
    cos_phi = np.cos(roll)
    sin_theta = np.sin(pitch)
    cos_theta = np.cos(pitch)
    sin_psi = np.sin(yaw)
    cos_psi = np.cos(yaw)

    # calculate transformation matrix
    C = np.matrix(np.zeros((3, 3)))
    C[0, 0] = cos_theta * cos_psi
    C[0, 1] = cos_theta * sin_psi
    C[0, 2] = -sin_theta
    C[1, 0] = -cos_phi * sin_psi + sin_phi * sin_theta * cos_psi
    C[1, 1] = cos_phi * cos_psi + sin_phi * sin_theta * sin_psi
    C[1, 2] = sin_phi * cos_theta
    C[2, 0] = sin_phi * sin_psi + cos_phi * sin_theta * cos_psi
    C[2, 1] = -sin_phi * cos_psi + cos_phi * sin_theta * sin_psi
    C[2, 2] = cos_phi * cos_theta

    return C


def ned_to_ecef_position(latitude, longitude, altitude):
    # Calculate transverse radius of curvature
    R_E = earth_radius / np.sqrt(1 - (eccentricity * np.sin(latitude)) ** 2)

    # convert position
    cos_lat = np.cos(latitude)
    sin_lat = np.sin(latitude)
    cos_long = np.cos(longitude)
    sin_long = np.sin(longitude)
    position_ecef = np.matrix([(R_E + altitude) * cos_lat * cos_long,
                               (R_E + altitude) * cos_lat * sin_long,
                               ((
                                    1 - eccentricity ** 2) * R_E + altitude) * sin_lat])

    return position_ecef


def ned_to_ecef_general(north, east, down):
    cos_lat = np.cos(north)
    sin_lat = np.sin(north)
    cos_long = np.cos(east)
    sin_long = np.sin(east)
    # Calculate ECEF to NED coordinate transformation matrix
    c_e_n = np.matrix([-sin_lat * cos_long, -sin_lat * sin_long, cos_lat,
                       -sin_long, cos_long, 0,
                       -cos_lat * cos_long, -cos_lat * sin_long, -sin_lat])

    # transform velocity
    velocity_ecef = c_e_n.T * np.matrix([north, east, down])

    return velocity_ecef


def ecef_to_ned(x, y, z):
    pass

def gravity_ecef(x, y, z):
    pass