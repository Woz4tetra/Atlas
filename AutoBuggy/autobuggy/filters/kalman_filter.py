import numpy as np
from numpy import linalg
import math
import navpy

#np.set_printoptions(precision=4)

earth_radius = 6378137  # WGS84 Equatorial radius in meters
earth_rotation_rate = 7.292115E-5  # rad/s
eccentricity = 0.0818191908425  # WGS84 eccentricity
earth_gravity_constant = 3.986004418E14
J_2 = 1.082627E-3


class GrovesKalmanFilter:
    def __init__(self, initial_roll_ecef, initial_pitch_ecef, initial_yaw_ecef,
                 roll_error, pitch_error, yaw_error,
                 initial_lat, initial_long, initial_alt,
                 initial_v_north, initial_v_east, initial_v_down,

                 initial_attitude_unc, initial_velocity_unc,
                 initial_position_unc, initial_accel_bias_unc,
                 initial_gyro_bias_unc, initial_clock_offset_unc,
                 initial_clock_drift_unc,

                 gyro_noise_PSD, accel_noise_PSD, accel_bias_PSD, gyro_bias_PSD,
                 pos_meas_SD, vel_meas_SD):
        self.static_properties = StaticProperties(
            initial_roll_ecef, initial_pitch_ecef, initial_yaw_ecef,
            roll_error, pitch_error, yaw_error,
            initial_lat, initial_long, initial_alt,
            initial_v_north, initial_v_east, initial_v_down,

            initial_attitude_unc, initial_velocity_unc,
            initial_position_unc, initial_accel_bias_unc,
            initial_gyro_bias_unc, initial_clock_offset_unc,
            initial_clock_drift_unc,

            gyro_noise_PSD, accel_noise_PSD, accel_bias_PSD, gyro_bias_PSD,
            pos_meas_SD, vel_meas_SD
        )

        self.frame_properties = CoordinateFrame(self.static_properties)
        print(self.get_position())

        self.ins = INS(self.frame_properties)
        self.epoch = Epoch(self.static_properties, self.frame_properties)

    def imu_updated(self, imu_dt, ax, ay, az, gx, gy, gz):
        # print("imu before:", self.get_position())
        self.frame_properties.state_updated(self.ins.update(
            imu_dt, np.matrix([ax, ay, az]).T, np.matrix([gx, gy, gz]).T))
        # print("imu after:", self.get_position())

    def gps_updated(self, gps_dt, lat, long, altitude):
        # print("gps before:", self.get_position())
        gps_position_ecef, gps_velocity_ecef = \
            self.frame_properties.get_gps_ecef(gps_dt, lat, long, altitude)

        self.frame_properties.state_updated(self.epoch.update(
            gps_dt, gps_position_ecef, gps_velocity_ecef,
            self.ins.accel_measurement
        ))
        # print("gps after:", self.get_position())

    def get_position(self):
        return navpy.ecef2lla(
            np.array(self.frame_properties.estimated_position.T)[0])
            
    def get_orientation(self):
        return navpy.dcm2angle(self.frame_properties.estimated_attitude)  # yaw, pitch, roll


class StaticProperties:
    def __init__(self, initial_roll, initial_pitch, initial_yaw,
                 roll_error, pitch_error, yaw_error,
                 initial_lat, initial_long, initial_alt,
                 initial_v_north, initial_v_east, initial_v_down,

                 initial_attitude_unc, initial_velocity_unc,
                 initial_position_unc, initial_accel_bias_unc,
                 initial_gyro_bias_unc, initial_clock_offset_unc,
                 initial_clock_drift_unc,

                 gyro_noise_PSD, accel_noise_PSD, accel_bias_PSD, gyro_bias_PSD,
                 pos_meas_SD, vel_meas_SD):
        # initialize properties (from in_profile)
        self.initial_lat = initial_lat  # Column 2
        self.initial_long = initial_long  # Column 3
        self.initial_alt = initial_alt  # Column 4
        self.initial_v_north = initial_v_north  # Column 5
        self.initial_v_east = initial_v_east  # Column 6
        self.initial_v_down = initial_v_down  # Column 7
        self.initial_roll = initial_roll  # Column 8
        self.initial_pitch = initial_pitch  # Column 9
        self.initial_yaw = initial_yaw  # Column 10

        # initialization_errors.delta_eul_nb_n
        self.initial_imu_errors = np.matrix(
            [roll_error, pitch_error, yaw_error])

        # properties of LC_KF_config
        self.uncertainty_values = dict(
            attitude=initial_attitude_unc,  # init_att_unc
            velocity=initial_velocity_unc,  # init_vel_unc
            position=initial_position_unc,  # init_pos_unc

            accel_bias=initial_accel_bias_unc,  # init_b_a_unc
            gyro_bias=initial_gyro_bias_unc,  # init_b_g_unc
            gyro_noise_PSD=gyro_noise_PSD,
            accel_noise_PSD=accel_noise_PSD,
            accel_bias_PSD=accel_bias_PSD,
            gyro_bias_PSD=gyro_bias_PSD,
            pos_meas_SD=pos_meas_SD,
            vel_meas_SD=vel_meas_SD,
        )

        # GNSS_config
        self.initial_clock_offset_unc = initial_clock_offset_unc  # rx_clock_offset
        self.initial_clock_drift_unc = initial_clock_drift_unc  # rx_clock_drift


class CoordinateFrame:
    def __init__(self, static_properties):
        self.static_properties = static_properties

        self.lat_ref = self.static_properties.initial_lat
        self.long_ref = self.static_properties.initial_long
        self.alt_ref = self.static_properties.initial_alt

        # Outputs of the Kalman filter

        # in NED, starting point is relative
        self.estimated_position = np.matrix(navpy.lla2ecef(
            self.lat_ref, self.long_ref, self.alt_ref)).T
        
        # prev gps variables
        self.prev_gps_position_ecef = np.matrix(navpy.lla2ecef(
            self.lat_ref, self.long_ref, self.alt_ref)).T

        self.estimated_velocity = self.ned_to_ecef(
            self.static_properties.initial_v_north,
            self.static_properties.initial_v_east,
            self.static_properties.initial_v_down
        )  # initial velocity should be given in meters with respect to your NED

        # old_est_C_b_e in Loosely_coupled_INS_GNSS (line 166)
        self.estimated_attitude = navpy.angle2dcm(
            self.static_properties.initial_yaw,
            self.static_properties.initial_pitch,
            self.static_properties.initial_roll,
        )

    def ned_to_ecef(self, north, east, down):
        return np.matrix(navpy.ned2ecef(
            (north, east, down),
            self.lat_ref, self.long_ref, self.alt_ref)).T

    def lla_to_ecef(self, lat, long, alt):
        return np.matrix(navpy.lla2ecef(lat, long, alt)).T

    def gps_v_ecef(self, gps_dt, lat, long, alt):
        ecef_position = self.lla_to_ecef(lat, long, alt)
        ecef_velocity = (ecef_position - self.prev_gps_position_ecef) / gps_dt
        self.prev_gps_position_ecef = ecef_position

        return ecef_velocity

    def get_gps_ecef(self, gps_dt, lat, long, altitude):
        # convert lat, long, altitude to position and velocity in ECEF
        gps_position_ecef = self.lla_to_ecef(lat, long, altitude)
        gps_velocity_ecef = self.gps_v_ecef(gps_dt, lat, long, altitude)

        return gps_position_ecef, gps_velocity_ecef

    def state_updated(self, state):
        self.estimated_position = state[0]
        self.estimated_velocity = state[1]
        self.estimated_attitude = state[2]


class INS:
    def __init__(self, frame_properties):
        self.frame_properties = frame_properties

        self.accel_measurement = np.matrix(np.zeros((3, 1)))

    def get_earth_rotation_matrix(self, dt):
        earth_rotation_amount = earth_rotation_rate * dt
        earth_rotation_matrix = np.matrix(
            [[np.cos(earth_rotation_amount), np.sin(earth_rotation_amount), 0],
             [-np.sin(earth_rotation_amount), np.cos(earth_rotation_amount), 0],
             [0, 0, 1]])

        return earth_rotation_amount, earth_rotation_matrix

    def get_gyro_skew_matrix(self, dt, gyro_measurement):
        # calculate attitude increment, magnitude, and skew-symmetric matrix
        angle_change_gyro = gyro_measurement * dt

        mag_angle_change_squared = angle_change_gyro.T * angle_change_gyro
        mag_angle_change = unit_to_scalar(
            np.sqrt(mag_angle_change_squared))  # convert to scalar
        skew_gyro_body = skew_symmetric(angle_change_gyro)

        return mag_angle_change, skew_gyro_body  # was skew_alpha_ib_b

    def get_estimated_attitude(self, mag_angle_change, skew_gyro_body,
                               earth_rotation_matrix, estimated_attitude):
        if mag_angle_change > 1E-8:
            C_new_old = (
                ((np.eye(3) + np.sin(mag_angle_change) / mag_angle_change) * (
                    skew_gyro_body)) +
                (((1 - np.cos(
                    mag_angle_change)) / mag_angle_change ** 2 * skew_gyro_body) * (
                     skew_gyro_body)))
        else:
            C_new_old = np.eye(3) + skew_gyro_body

        return earth_rotation_matrix * (estimated_attitude * C_new_old)

    def get_average_attitude(self, mag_angle_change, estimated_attitude,
                             skew_gyro_body, earth_rotation_amount):
        if mag_angle_change > 1E-8:
            average_attitude = (
                estimated_attitude * (
                    np.matrix(np.eye(3)) + (
                        1 - np.cos(mag_angle_change)) / mag_angle_change ** 2
                    * skew_gyro_body + (
                        1 - np.sin(
                            mag_angle_change) / mag_angle_change) / mag_angle_change ** 2
                    * skew_gyro_body * skew_gyro_body) -
                0.5 * skew_symmetric(
                    np.matrix([0, 0, earth_rotation_amount]).T) *
                estimated_attitude)
        else:
            average_attitude = \
                (estimated_attitude - 0.5 * skew_symmetric(
                    np.matrix([0, 0, earth_rotation_amount]).T) *
                 estimated_attitude)

        return average_attitude

    def update(self, dt, accel_measurement, gyro_measurement):
        self.accel_measurement = accel_measurement
        # Nav_equations_ECEF function

        # this section is attitude update
        earth_rotation_amount, earth_rotation_matrix = \
            self.get_earth_rotation_matrix(dt)
        mag_angle_change, skew_gyro_body = \
            self.get_gyro_skew_matrix(dt, gyro_measurement)
        estimated_attitude = \
            self.get_estimated_attitude(
                mag_angle_change, skew_gyro_body, earth_rotation_matrix,
                self.frame_properties.estimated_attitude
            )
        average_attitude = self.get_average_attitude(
            mag_angle_change, estimated_attitude, skew_gyro_body,
            earth_rotation_amount
        )

        # Transform specific force to ECEF-frame resolving axes
        # TODO: average_attitude -> transform
        accel_meas_ecef = average_attitude * self.accel_measurement

        # update velocity
        # TODO: prev_est_p -> prev_est_r
        prev_est_p = self.frame_properties.estimated_position
        prev_est_v = self.frame_properties.estimated_velocity

        estimated_velocity = \
            prev_est_v + dt * (
                accel_meas_ecef - 2 * skew_symmetric(
                    np.matrix([0, 0, earth_rotation_rate]).T) * prev_est_v)
        # update cartesian position
        estimated_position = \
            prev_est_p + (estimated_velocity + prev_est_v) * 0.5 * dt
        
        return estimated_position, estimated_velocity, estimated_attitude


class Epoch:
    def __init__(self, static_properties, frame_properties):
        self.static_properties = static_properties
        self.frame_properties = frame_properties

        self.skew_earth_rotation = skew_symmetric(
            np.matrix([0, 0, earth_rotation_rate]).T)

        # P_matrix & est_IMU_bias in Loosely_coupled_INS_GNSS (line 192)
        self.error_covariance_P = self.init_P()
        self.estimated_imu_biases = np.matrix(np.zeros((6, 1)))
        self.state_transition_Phi = None  # Phi_matrix

        self.error_covariance_propagated_P = None
        self.approx_noise_covariance_Q = None
        self.measurement_model_H = None
        self.noise_covariance_R = None
        self.kalman_gain_K = None
        # TODO: measurement_innovation_Z -> delta_z
        self.measurement_innovation_Z = np.matrix(np.zeros((6, 1)))

    def init_P(self):
        error_covariance_P = np.matrix(np.zeros((15, 15)))

        I_3 = np.matrix(np.eye(3))
        error_covariance_P[0:3, 0:3] = \
            I_3 * self.static_properties.uncertainty_values["attitude"] ** 2
        error_covariance_P[3:6, 3:6] = \
            I_3 * self.static_properties.uncertainty_values["velocity"] ** 2
        error_covariance_P[6:9, 6:9] = \
            I_3 * self.static_properties.uncertainty_values["position"] ** 2
        error_covariance_P[9:12, 9:12] = \
            I_3 * self.static_properties.uncertainty_values["accel_bias"] ** 2
        error_covariance_P[12:15, 12:15] = \
            I_3 * self.static_properties.uncertainty_values["gyro_bias"] ** 2
        # error_covariance_P[15, 15] = \
        #     self.static_properties.initial_clock_offset_unc ** 2
        # error_covariance_P[16, 16] = \
        #     self.static_properties.initial_clock_drift_unc ** 2

        return error_covariance_P

    def update(self, dt, gps_position_ecef, gps_velocity_ecef,
               accel_measurement):
        self.determine_transition_matrix(dt, accel_measurement)  # step 1
        self.determine_noise_covariance(
            dt, self.static_properties.uncertainty_values["gyro_noise_PSD"],
            self.static_properties.uncertainty_values["accel_noise_PSD"],
            self.static_properties.uncertainty_values["accel_bias_PSD"],
            self.static_properties.uncertainty_values["gyro_bias_PSD"]
        )  # step 2
        self.set_propagated_state_est()  # step 3
        self.set_propagated_error_est()  # step 4
        self.setup_measurement_matrix()  # step 5

        self.setup_noise_covariance_matrix(
            self.static_properties.uncertainty_values["pos_meas_SD"],
            self.static_properties.uncertainty_values["vel_meas_SD"],
        )  # step 6
        self.calc_kalman_gain()  # step 7
        self.formulate_measurement_innovations(
            gps_position_ecef, gps_velocity_ecef,
            self.frame_properties.estimated_position,
            self.frame_properties.estimated_velocity
        )  # step 8
        self.update_state_estimates()  # step 9
        self.update_covariance_matrix()  # step 10

        estimated_position, estimated_velocity, estimated_attitude, estimated_imu_biases = \
            self.correct_estimates(
                self.frame_properties.estimated_attitude,
                self.frame_properties.estimated_position,
                self.frame_properties.estimated_velocity,
                self.estimated_imu_biases,
            )
        return estimated_position, estimated_velocity, estimated_attitude

    def determine_transition_matrix(self, dt, accel_measurement):
        # step 1
        est_p = self.frame_properties.estimated_position
        to_ecef = self.frame_properties.estimated_attitude

        estimated_lat = \
            navpy.ecef2lla(vector_to_list(est_p), latlon_unit='rad')[0]

        self.state_transition_Phi = np.matrix(np.eye(15))

        self.state_transition_Phi[0:3, 0:3] -= self.skew_earth_rotation * dt
        self.state_transition_Phi[0:3, 12:15] = to_ecef * dt
        self.state_transition_Phi[3:6, 0:3] = \
            -dt * skew_symmetric(to_ecef * accel_measurement)

        self.state_transition_Phi[3:6, 3:6] -= 2 * self.skew_earth_rotation * dt

        geocentric_radius = \
            earth_radius / np.sqrt(
                1 - (eccentricity * np.sin(estimated_lat)) ** 2) * np.sqrt(
                np.cos(estimated_lat) ** 2 + (
                    1 - eccentricity ** 2) ** 2 * np.sin(
                    estimated_lat) ** 2)

        self.state_transition_Phi[3:6, 6:9] = \
            (-dt * 2 * gravity_ecef(
                est_p) / geocentric_radius * est_p.T / np.sqrt(
                unit_to_scalar(est_p.T * est_p)))

        self.state_transition_Phi[3:6, 9:12] = est_p * dt

        self.state_transition_Phi[6:9, 3:6] = np.matrix(np.eye(3)) * dt

    def determine_noise_covariance(self, dt, gyro_noise_PSD, accel_noise_PSD,
                                   accel_bias_PSD, gyro_bias_PSD):
        # step 2
        # Q_prime_matrix in the matlab code
        self.approx_noise_covariance_Q = np.matrix(np.eye(15)) * dt

        self.approx_noise_covariance_Q[0:3, 0:3] *= gyro_noise_PSD
        self.approx_noise_covariance_Q[3:6, 3:6] *= accel_noise_PSD
        self.approx_noise_covariance_Q[9:12, 9:12] *= accel_bias_PSD
        self.approx_noise_covariance_Q[12:15, 12:15] *= gyro_bias_PSD

    def set_propagated_state_est(self):
        # step 3
        self.estimated_state_prop_x = np.matrix(np.zeros((15, 1)))
        # x_est_propagated in the matlab code

    def set_propagated_error_est(self):
        # step 4
        self.error_covariance_propagated_P = self.state_transition_Phi * (
            self.error_covariance_P + 0.5 * self.approx_noise_covariance_Q) * \
                                             self.state_transition_Phi.T + \
                                             0.5 * self.approx_noise_covariance_Q
        # self.error_covariance_propagated_P = (
        #     (self.state_transition_Phi *
        #      self.error_covariance_P) * self.state_transition_Phi.T +
        #     self.approx_noise_covariance_Q
        # )

    def setup_measurement_matrix(self):
        # step 5
        # H_matrix in the matlab code
        self.measurement_model_H = np.matrix(np.zeros((6, 15)))
        self.measurement_model_H[0:3, 6:9] = -np.matrix(np.eye(3))
        self.measurement_model_H[3:6, 3:6] = -np.matrix(np.eye(3))

    def setup_noise_covariance_matrix(self, pos_meas_SD, vel_meas_SD):
        # step 6
        # R_matrix in the matlab code
        self.noise_covariance_R = np.matrix(np.eye(6))

        self.noise_covariance_R[0:3, 0:3] *= pos_meas_SD ** 2
        self.noise_covariance_R[3:6, 3:6] *= vel_meas_SD ** 2

        # self.noise_covariance_R[0:3, 0:3] = \
        #     np.matrix(np.eye(3)) * pos_meas_SD ** 2
        # self.noise_covariance_R[0:3, 3:6] = np.matrix(np.zeros((3, 3)))
        # self.noise_covariance_R[3:6, 0:3] = np.matrix(np.zeros((3, 3)))
        # self.noise_covariance_R[3:6, 3:6] = \
        #     np.matrix(np.eye(3)) * vel_meas_SD ** 2

    def calc_kalman_gain(self):
        # step 7
        self.kalman_gain_K = self.error_covariance_propagated_P * \
                             self.measurement_model_H.T * linalg.inv(
            (self.measurement_model_H * self.error_covariance_propagated_P) *
            self.measurement_model_H.T + self.noise_covariance_R)

    def formulate_measurement_innovations(self, gps_position_ecef,
                                          gps_velocity_ecef, estimated_position,
                                          estimated_velocity):
        # step 8
        # assuming zero lever arm
        self.measurement_innovation_Z[0:3] = \
            gps_position_ecef - estimated_position
        self.measurement_innovation_Z[3:6] = \
            gps_velocity_ecef - estimated_velocity

    def update_state_estimates(self):
        # step 9
        # x_est_propagated in the matlab code
        self.estimated_state_prop_x += self.kalman_gain_K * \
                                       self.measurement_innovation_Z

    def update_covariance_matrix(self):
        # step 10
        # P_matrix_new, P_matrix in the matlab code
        self.error_covariance_P = (np.matrix(
            np.eye(15)) - self.kalman_gain_K * self.measurement_model_H) * \
                                  self.error_covariance_propagated_P

    def correct_estimates(self, estimated_attitude, estimated_position,
                          estimated_velocity, estimated_imu_biases):
        estimated_attitude = ((np.matrix(
            np.eye(3)) - skew_symmetric(self.estimated_state_prop_x[0:3])) *
                              estimated_attitude)
        estimated_velocity -= self.estimated_state_prop_x[3:6]
        estimated_position -= self.estimated_state_prop_x[6:9]
        estimated_imu_biases += self.estimated_state_prop_x[9:15]

        return estimated_position, estimated_velocity, estimated_attitude, \
               estimated_imu_biases


def skew_symmetric(m):
    """
    creates a 3v3 skew_symmetric matrix from a 1x3 matrix
    """
    return np.matrix([[0, -m[2], m[2]],
                      [m[2], 0, -m[0]],
                      [-m[1], m[0], 0]])


def euler_to_ctm(orientation):
    """
    Purpose
    ----------
    Converts a set of Euler angles to the corresponding coordinate transformation matrix

    Parameters
    ----------
    orientation: Euler angles describing the rotation from beta to alpha in the order (roll, pitch, yaw)

    Returns
    ----------
    C: coordinate transformation matrix describing transformation from beta to alpha

    """
    roll, pitch, yaw = vector_to_list(orientation)

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


def gravity_ecef(ecef_vector):
    # Calculate distance from center of the Earth
    mag_r = unit_to_scalar(np.sqrt(ecef_vector.T * ecef_vector))

    # If the input position is 0,0,0, produce a dummy output
    if mag_r == 0:
        return np.matrix(np.zeros((3, 1)))

    # Calculates gravitational acceleration
    else:
        z_scale = unit_to_scalar((ecef_vector[2] / mag_r) ** 2 * 5)

        matrix = np.matrix([(1 - z_scale) * unit_to_scalar(ecef_vector[0]),
                            (1 - z_scale) * unit_to_scalar(ecef_vector[1]),
                            (3 - z_scale) * unit_to_scalar(ecef_vector[2])])
        gamma = np.matrix(-earth_gravity_constant / mag_r ** 3 *
                          (ecef_vector + 1.5 * J_2 * (
                              earth_radius / mag_r) ** 2 * matrix))
        # Add centripetal acceleration
        g = np.matrix([unit_to_scalar(
            gamma[0]) + earth_rotation_rate ** 2 * unit_to_scalar(
            ecef_vector[0]),
                       unit_to_scalar(gamma[
                                          1]) + earth_rotation_rate ** 2 * unit_to_scalar(
                           ecef_vector[1]),
                       unit_to_scalar(gamma[2])]).T
        return g


def unit_to_scalar(unit_matrix):
    return unit_matrix.tolist()[0][0]


def vector_to_list(vector, is_column_vector=True):
    if not is_column_vector:
        return vector.tolist()[0]
    else:
        return vector.T.tolist()[0]

def get_gps_orientation(lat1, long1, alt1, lat2, long2, alt2):
        pos1 = navpy.lla2ecef(lat1, long1, alt1)
        pos2 = navpy.lla2ecef(lat2, long2, alt2)
        
        delta_pos = pos2 - pos1
        
        yaw = math.atan2(delta_pos[1], delta_pos[0])
        pitch = math.atan2(delta_pos[2] * math.cos(yaw), delta_pos[0])
        roll = math.atan2(math.cos(yaw), math.sin(yaw) * math.sin(pitch))
        
        return yaw, pitch, roll

if __name__ == '__main__':
    def almost_equal(float1, float2, epsilon=0.001):
        if abs(float2 - float1) > epsilon:
            raise ValueError("Floats aren't equal: %f, %f" % (float1, float2))
        else:
            return True


    def test_coordinate_transforms(lat1, long1, alt1):
        ecef_position = navpy.lla2ecef(lat1, long1, alt1)

        # print(ecef_position)

        lat2, long2, alt2 = navpy.ecef2lla(ecef_position)

        # print(lat2, long2, alt2)

        almost_equal(lat1, lat2)
        almost_equal(long1, long2)
        almost_equal(alt1, alt2)


    def test_helpers():
        lat1, long1, alt1 = 40.4404285, -79.9422232, 296.18
        ecef_position = navpy.lla2ecef(lat1, long1, alt1)

        x, y, z = ecef_position
        almost_equal(x, 848993, epsilon=0.5)
        almost_equal(y, -4786645, epsilon=0.5)
        almost_equal(z, 4115520, epsilon=0.5)

        test_coordinate_transforms(
            40.44057846069336, 79.94245147705078, 302.79998779296875)


    def test_angle_transform():
        g = 9.80665

        static_properties = StaticProperties(
            initial_lat=40.44057846069336,
            initial_long=79.94245147705078,
            initial_alt=302.79998779296875,

            initial_roll=0,
            initial_pitch=0,
            initial_yaw=0,

            roll_error=math.radians(-0.05),
            pitch_error=math.radians(0.04),
            yaw_error=math.radians(1),

            initial_v_north=0,
            initial_v_east=0,
            initial_v_down=0,

            initial_attitude_unc=math.radians(1),
            initial_velocity_unc=0.1,
            initial_position_unc=10,
            initial_accel_bias_unc=(g * 1E-3) ** 2,
            initial_gyro_bias_unc=math.radians(1 / 3600),
            initial_clock_offset_unc=10000,
            initial_clock_drift_unc=100,

            gyro_noise_PSD=math.radians(0.02 / 60) ** 2,
            accel_noise_PSD=200 * (g * 1E-6) ** 2,
            accel_bias_PSD=1.0E-7,
            gyro_bias_PSD=2.0E-12,
            pos_meas_SD=2.5,
            vel_meas_SD=0.1
        )

        frame_properties = CoordinateFrame(static_properties)
        ins = INS(frame_properties)

        gyro_measurement = np.matrix([0, 0, math.pi / 2]).T
        accel_measurement = np.matrix([1, 0, 0]).T

        dt = 0.001

        earth_rotation_amount, earth_rotation_matrix = \
            ins.get_earth_rotation_matrix(dt)
        mag_angle_change, skew_gyro_body = \
            ins.get_gyro_skew_matrix(dt, gyro_measurement)
        estimated_attitude = \
            ins.get_estimated_attitude(
                mag_angle_change, skew_gyro_body, earth_rotation_matrix,
                frame_properties.estimated_attitude
            )
        average_attitude = ins.get_average_attitude(
            mag_angle_change, estimated_attitude, skew_gyro_body,
            earth_rotation_amount
        )

        accel_meas_ecef = average_attitude * accel_measurement

        print("accel_meas_ecef")
        print(accel_meas_ecef, linalg.norm(accel_meas_ecef))

        print("estimated_attitude")
        print(estimated_attitude, linalg.norm(estimated_attitude))

        print("average_attitude")
        print(average_attitude, linalg.norm(average_attitude))

        print("mag_angle_change")
        print(mag_angle_change)

        prev_est_p = frame_properties.estimated_position
        prev_est_v = frame_properties.estimated_velocity

        estimated_velocity = \
            prev_est_v + dt * (
                accel_meas_ecef - 2 * skew_symmetric(
                    np.matrix([0, 0, earth_rotation_rate]).T) * prev_est_v)
        # update cartesian position
        estimated_position = \
            prev_est_p + (estimated_velocity + prev_est_v) * 0.5 * dt

        delta_position = \
            estimated_position - np.matrix(
                navpy.lla2ecef(static_properties.initial_lat,
                               static_properties.initial_long,
                               static_properties.initial_alt)).T
        print("estimated_position")
        print(delta_position, linalg.norm(delta_position))

        print("estimated_velocity")
        print(estimated_velocity, linalg.norm(estimated_velocity))


    def test_all():
        test_helpers()
        test_angle_transform()


    test_all()
