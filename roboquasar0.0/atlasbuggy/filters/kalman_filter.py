import numpy as np
from numpy import linalg
import navpy

# np.set_printoptions(precision=4)

earth_radius = 6378137  # WGS84 Equatorial radius in meters
earth_rotation_rate = 7.292115E-5  # rad/s
eccentricity = 0.0818191908425  # WGS84 eccentricity
earth_gravity_constant = 3.986004418E14
J_2 = 1.082627E-3


class GrovesKalmanFilter:
    def __init__(self, **kf_properties):
        self.properties = KalmanProperties(**kf_properties)

        self.ins = INS(self.properties)
        self.epoch = Epoch(self.properties)

        self.is_active = False

    def imu_updated(self, imu_dt, ax, ay, az, gx, gy, gz):
        if imu_dt > 0:
            self.properties.estimated_position, \
            self.properties.estimated_velocity, \
            self.properties.estimated_attitude = self.ins.update(
                imu_dt, np.matrix([ax, ay, az]).T, np.matrix([gx, gy, gz]).T,
            )

    def gps_updated(self, gps_dt, lat, long, altitude):
        if gps_dt > 0:
            gps_position_ecef, gps_velocity_ecef = \
                self.properties.get_gps_ecef(gps_dt, lat, long, altitude)

            self.properties.estimated_position, \
            self.properties.estimated_velocity, \
            self.properties.estimated_attitude, \
            self.properties.estimated_imu_biases = self.epoch.update(
                gps_dt, gps_position_ecef, gps_velocity_ecef,
                self.ins.accel_measurement
            )
            self.is_active = True

    def get_position(self):
        return navpy.ecef2lla(
            np.array(self.properties.estimated_position.T)[0])

    def get_orientation(self):
        return navpy.dcm2angle(  # CTM_to_Euler
            self.properties.estimated_attitude)  # yaw, pitch, roll


class KalmanProperties:
    def __init__(self, initial_roll, initial_pitch, initial_yaw,
                 initial_lat, initial_long, initial_alt,
                 initial_v_north, initial_v_east, initial_v_down,

                 initial_attitude_unc, initial_velocity_unc,
                 initial_position_unc, initial_accel_bias_unc,
                 initial_gyro_bias_unc,

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

        self.lat_ref = self.initial_lat
        self.long_ref = self.initial_long
        self.alt_ref = self.initial_alt

        # Outputs of the Kalman filter

        self.estimated_position = np.matrix(navpy.lla2ecef(
            self.lat_ref, self.long_ref, self.alt_ref)).T

        # prev gps variables
        self.prev_gps_position_ecef = np.matrix(navpy.lla2ecef(
            self.lat_ref, self.long_ref, self.alt_ref)).T

        self.estimated_velocity = self.ned_to_ecef(
            self.initial_v_north,
            self.initial_v_east,
            self.initial_v_down
        )  # initial velocity should be given in meters with respect to your NED

        # old_est_C_b_e in Loosely_coupled_INS_GNSS (line 166)
        self.estimated_attitude = np.matrix(navpy.angle2dcm(  # Euler_to_CTM
            self.initial_yaw,
            self.initial_pitch,
            self.initial_roll,
        ))

        self.estimated_imu_biases = np.matrix(np.zeros((6, 1)))

    def ned_to_ecef(self, north, east, down):
        return np.matrix(navpy.ned2ecef(
            (north, east, down),
            self.lat_ref, self.long_ref, self.alt_ref)).T

    def lla_to_ecef(self, lat, long, alt):
        return np.matrix(navpy.lla2ecef(lat, long, alt)).T

    def gps_velocity_ecef(self, gps_dt, lat, long, alt):
        ecef_position = self.lla_to_ecef(lat, long, alt)
        ecef_velocity = (ecef_position - self.prev_gps_position_ecef) / gps_dt
        self.prev_gps_position_ecef = ecef_position

        return ecef_velocity

    def get_gps_ecef(self, gps_dt, lat, long, altitude):
        # convert lat, long, altitude to position and velocity in ECEF
        gps_position_ecef = self.lla_to_ecef(lat, long, altitude)
        gps_velocity_ecef = self.gps_velocity_ecef(gps_dt, lat, long, altitude)

        return gps_position_ecef, gps_velocity_ecef


class INS:
    def __init__(self, properties: KalmanProperties):
        self.properties = properties

        self.accel_measurement = np.matrix(np.zeros((3, 1)))
        self.gyro_measurement = np.matrix(np.zeros((3, 1)))

        use_rigorous_update = False

        if use_rigorous_update:
            self.update = self.rigorous_update
            print("Using RIGOROUS update")
        else:
            self.update = self.non_rigorous_update
            print("Using NON-RIGOROUS update")

    def get_earth_rotation_transform(self, dt):
        earth_rotation_amount = earth_rotation_rate * dt
        earth_rotation_transform = np.matrix(
            [[np.cos(earth_rotation_amount), np.sin(earth_rotation_amount), 0],
             [-np.sin(earth_rotation_amount), np.cos(earth_rotation_amount), 0],
             [0, 0, 1]])

        return earth_rotation_amount, earth_rotation_transform

    def get_gyro_skew_matrix(self, dt, gyro_measurement):
        # calculate attitude increment, magnitude, and skew-symmetric matrix
        angle_change_gyro = gyro_measurement * dt

        mag_angle_change = unit_to_scalar(np.sqrt(
            angle_change_gyro.T * angle_change_gyro))  # convert to scalar

        skew_gyro_body = skew_symmetric(angle_change_gyro)

        return mag_angle_change, skew_gyro_body  # was skew_alpha_ib_b

    def get_estimated_attitude(self, mag_angle_change, skew_gyro_body,
                               earth_rotation_transform,
                               estimated_attitude):
        if mag_angle_change > 1E-8:
            estimated_attitude_new = (
                np.matrix(np.eye(3)) + np.sin(mag_angle_change) /
                mag_angle_change * skew_gyro_body +
                (1 - np.cos(mag_angle_change)) /
                mag_angle_change ** 2 *
                skew_gyro_body * skew_gyro_body)
        else:
            estimated_attitude_new = np.matrix(np.eye(3)) + skew_gyro_body

        # check the order of operations here
        return (earth_rotation_transform * estimated_attitude) * \
               estimated_attitude_new

    def get_average_attitude(self, mag_angle_change, estimated_attitude,
                             skew_gyro_body, earth_rotation_amount_skew):
        if mag_angle_change > 1E-8:
            average_attitude = (
                (estimated_attitude *
                 (np.matrix(np.eye(3)) + (1 - np.cos(mag_angle_change)) /
                  mag_angle_change ** 2 * skew_gyro_body +
                  (1 - np.sin(mag_angle_change) / mag_angle_change) /
                  mag_angle_change ** 2 * skew_gyro_body * skew_gyro_body) -
                 0.5 * earth_rotation_amount_skew *
                 estimated_attitude))
        else:
            average_attitude = \
                (estimated_attitude - 0.5 * earth_rotation_amount_skew *
                 estimated_attitude)

        return average_attitude

    def rigorous_update(self, dt, accel_measurement, gyro_measurement):
        self.modify_measurements(accel_measurement, gyro_measurement)

        # Nav_equations_ECEF function

        # this section is attitude update
        earth_rotation_amount, earth_rotation_transform = \
            self.get_earth_rotation_transform(dt)
        earth_rotation_amount_skew = skew_symmetric(
            np.matrix([0, 0, earth_rotation_amount]).T)

        mag_angle_change, skew_gyro_body = \
            self.get_gyro_skew_matrix(dt, self.gyro_measurement)
        estimated_attitude = \
            self.get_estimated_attitude(
                mag_angle_change, skew_gyro_body, earth_rotation_transform,
                self.properties.estimated_attitude
            )

        average_attitude_transform = self.get_average_attitude(
            mag_angle_change, estimated_attitude, skew_gyro_body,
            earth_rotation_amount_skew
        )

        prev_est_r = self.properties.estimated_position
        prev_est_v = self.properties.estimated_velocity

        # Transform specific force to ECEF-frame resolving axes
        accel_meas_ecef = average_attitude_transform * self.accel_measurement

        # update velocity
        # g = gravity_ecef(prev_est_r)

        # TODO: do we need gravity_ecef here?
        estimated_velocity = \
            prev_est_v + dt * (
                accel_meas_ecef  # + g
                - 2 * earth_rotation_amount_skew * prev_est_v)

        # update cartesian position
        estimated_position = \
            prev_est_r + (estimated_velocity + prev_est_v) * 0.5 * dt

        return estimated_position, estimated_velocity, estimated_attitude

    # ---- non rigorous update fn -----

    def modify_measurements(self, accel_measurement, gyro_measurement):
        # accel_measurement = np.clip(accel_measurement, -0.25, 0.25)

        self.accel_measurement = \
            accel_measurement - self.properties.estimated_imu_biases[0:3]
        self.gyro_measurement = \
            gyro_measurement - self.properties.estimated_imu_biases[3:6]

        self.accel_measurement = np.clip(self.accel_measurement, -0.5, 0.5)

    def non_rigorous_update(self, dt, accel_measurement, gyro_measurement):
        self.modify_measurements(accel_measurement, gyro_measurement)

        prev_r = self.properties.estimated_position
        prev_v = self.properties.estimated_velocity
        prev_C = self.properties.estimated_attitude

        # Nav_equations_ECEF function

        angle_change = -self.gyro_measurement * dt  # assuming constant between dt's

        skew_angle_change = skew_symmetric(angle_change)

        # now estimate attitude, not sure if right

        est_attitude_trans = prev_C * (np.matrix(np.eye(3)) + skew_angle_change)

        accel_ecef = est_attitude_trans * self.accel_measurement

        estimated_velocity = prev_v + dt * accel_ecef
        # left it as est_vel here so that i can use it and self.vel at the same time
        # the next step

        estimated_position = prev_r + (prev_v + estimated_velocity) * 0.5 * dt

        return estimated_position, estimated_velocity, est_attitude_trans


class Epoch:
    def __init__(self, properties: KalmanProperties):
        self.properties = properties

        self.skew_earth_rotation = skew_symmetric(
            np.matrix([0, 0, earth_rotation_rate]).T)

        # P_matrix & est_IMU_bias in Loosely_coupled_INS_GNSS (line 192)
        self.error_covariance_P = self.init_P()
        self.state_transition_Phi = None  # Phi_matrix

        self.error_covariance_propagated_P = None
        self.approx_noise_covariance_Q = None
        self.measurement_model_H = None
        self.noise_covariance_R = None
        self.kalman_gain_K = None
        self.measurement_innovation_delta_Z = np.matrix(np.zeros((6, 1)))

    def init_P(self):
        error_covariance_P = np.matrix(np.zeros((15, 15)))

        I_3 = np.matrix(np.eye(3))
        error_covariance_P[0:3, 0:3] = \
            I_3 * self.properties.uncertainty_values["attitude"] ** 2
        error_covariance_P[3:6, 3:6] = \
            I_3 * self.properties.uncertainty_values["velocity"] ** 2
        error_covariance_P[6:9, 6:9] = \
            I_3 * self.properties.uncertainty_values["position"] ** 2
        error_covariance_P[9:12, 9:12] = \
            I_3 * self.properties.uncertainty_values["accel_bias"] ** 2
        error_covariance_P[12:15, 12:15] = \
            I_3 * self.properties.uncertainty_values["gyro_bias"] ** 2

        return error_covariance_P

    def update(self, dt, gps_position_ecef, gps_velocity_ecef,
               accel_measurement):
        # step 1
        self.determine_transition_matrix(
            dt, accel_measurement,
            self.properties.estimated_position,
            self.properties.estimated_attitude
        )
        self.determine_noise_covariance(
            dt, self.properties.uncertainty_values["gyro_noise_PSD"],
            self.properties.uncertainty_values["accel_noise_PSD"],
            self.properties.uncertainty_values["accel_bias_PSD"],
            self.properties.uncertainty_values["gyro_bias_PSD"]
        )  # step 2
        self.set_propagated_state_est()  # step 3
        self.set_propagated_error_est()  # step 4
        self.setup_measurement_model_matrix()  # step 5

        self.setup_noise_covariance_matrix(
            self.properties.uncertainty_values["pos_meas_SD"],
            self.properties.uncertainty_values["vel_meas_SD"],
        )  # step 6
        self.calc_kalman_gain()  # step 7
        self.formulate_measurement_innovations(
            gps_position_ecef, gps_velocity_ecef,
            self.properties.estimated_position,
            self.properties.estimated_velocity
        )  # step 8
        self.update_state_estimates()  # step 9
        self.update_covariance_matrix()  # step 10

        return self.correct_estimates(
            self.properties.estimated_attitude,
            self.properties.estimated_position,
            self.properties.estimated_velocity,
            self.properties.estimated_imu_biases
        )

    def determine_transition_matrix(self, dt, accel_measurement,
                                    estimated_position, estimated_attitude):
        # step 1
        estimated_r = estimated_position
        est_body_ecef_trans = estimated_attitude

        estimated_lat = \
            navpy.ecef2lla(vector_to_list(estimated_r), latlon_unit='rad')[0]

        self.state_transition_Phi = np.matrix(np.eye(15))

        self.state_transition_Phi[0:3, 0:3] -= self.skew_earth_rotation * dt
        self.state_transition_Phi[0:3, 12:15] = est_body_ecef_trans * dt
        self.state_transition_Phi[3:6, 0:3] = \
            -dt * skew_symmetric(
                est_body_ecef_trans * accel_measurement)

        self.state_transition_Phi[3:6, 3:6] -= 2 * self.skew_earth_rotation * dt

        geocentric_radius = \
            earth_radius / np.sqrt(
                1 - (eccentricity * np.sin(estimated_lat)) ** 2) * np.sqrt(
                np.cos(estimated_lat) ** 2 + (
                    1 - eccentricity ** 2) ** 2 * np.sin(
                    estimated_lat) ** 2)

        self.state_transition_Phi[3:6, 6:9] = \
            (-dt * 2 * gravity_ecef(
                estimated_r) / (
                 geocentric_radius * estimated_lat) * estimated_r.T /
             unit_to_scalar(np.sqrt(estimated_r.T * estimated_r)))

        self.state_transition_Phi[3:6, 9:12] = est_body_ecef_trans * dt

        self.state_transition_Phi[6:9, 3:6] = np.matrix(np.eye(3)) * dt

    def determine_noise_covariance(self, dt, gyro_noise_PSD, accel_noise_PSD,
                                   accel_bias_PSD, gyro_bias_PSD):
        # step 2
        # Q_prime_matrix in the matlab code
        self.approx_noise_covariance_Q = np.matrix(np.eye(15)) * dt

        self.approx_noise_covariance_Q[0:3, 0:3] *= gyro_noise_PSD
        self.approx_noise_covariance_Q[3:6, 3:6] *= accel_noise_PSD
        self.approx_noise_covariance_Q[6:9, 6:9] = np.matrix(np.zeros((3, 3)))
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

    def setup_measurement_model_matrix(self):
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
            ((self.measurement_model_H * self.error_covariance_propagated_P) *
             self.measurement_model_H.T) + self.noise_covariance_R)

    def formulate_measurement_innovations(self, gps_position_ecef,
                                          gps_velocity_ecef, estimated_position,
                                          estimated_velocity):
        # step 8
        self.measurement_innovation_delta_Z[0:3] = \
            gps_position_ecef - estimated_position
        self.measurement_innovation_delta_Z[3:6] = \
            gps_velocity_ecef - estimated_velocity

    def update_state_estimates(self):
        # step 9
        # x_est_propagated in the matlab code
        self.estimated_state_prop_x += self.kalman_gain_K * \
                                       self.measurement_innovation_delta_Z

    def update_covariance_matrix(self):
        # step 10
        # P_matrix_new, P_matrix in the matlab code
        self.error_covariance_P = (np.matrix(
            np.eye(15)) - self.kalman_gain_K * self.measurement_model_H) * \
                                  self.error_covariance_propagated_P

    def correct_estimates(self, estimated_attitude, estimated_position,
                          estimated_velocity, estimated_imu_biases):
        estimated_attitude_new = (np.matrix(np.eye(3)) - skew_symmetric(
            self.estimated_state_prop_x[0:3])) * estimated_attitude
        estimated_velocity_new = \
            estimated_velocity - self.estimated_state_prop_x[3:6]
        estimated_position_new = \
            estimated_position - self.estimated_state_prop_x[6:9]
        estimated_imu_biases_new = \
            estimated_imu_biases + self.estimated_state_prop_x[9:15]

        return estimated_position_new, estimated_velocity_new, \
               estimated_attitude_new, estimated_imu_biases_new


def skew_symmetric(m):
    # current assumption is that the shapes are happy. just making a note in case things go weird later
    """
    creates a 3v3 skew_symmetric matrix from a 1x3 matrix
    """
    return np.matrix([[0, -m[2], m[1]],
                      [m[2], 0, -m[0]],
                      [-m[1], m[0], 0]])


def gravity_ecef(ecef_vector):
    # Calculate distance from center of the Earth
    mag_r = unit_to_scalar(np.sqrt(ecef_vector.T * ecef_vector))

    # If the input position is 0,0,0, produce a dummy output
    if mag_r == 0:
        return np.matrix(np.zeros((3, 1)))

    # Calculates gravitational acceleration in ECEF
    else:
        z_value = unit_to_scalar(ecef_vector[2])
        z_scale = 5 * ((z_value / mag_r) ** 2)

        matrix = np.matrix([(1 - z_scale) * unit_to_scalar(ecef_vector[0]),
                            (1 - z_scale) * unit_to_scalar(ecef_vector[1]),
                            (3 - z_scale) * unit_to_scalar(ecef_vector[2])])

        operation_1 = -earth_gravity_constant / mag_r ** 3
        operation_2 = 1.5 * J_2 * (earth_radius / mag_r) ** 2

        gamma = np.matrix(operation_1 * (ecef_vector + operation_2 * matrix))

        # Add centripetal acceleration
        g = np.matrix([
            unit_to_scalar(gamma[0]) + earth_rotation_rate ** 2 *
            unit_to_scalar(ecef_vector[0]),
            unit_to_scalar(gamma[1]) + earth_rotation_rate ** 2 *
            unit_to_scalar(ecef_vector[1]),
            unit_to_scalar(gamma[2])]
        ).T

        return g


def unit_to_scalar(unit_matrix: np.matrix):
    return unit_matrix.tolist()[0][0]


def vector_to_list(vector, is_column_vector=True):
    if not is_column_vector:
        return vector.tolist()[0]
    else:
        return vector.T.tolist()[0]


def get_gps_orientation(lat1, long1, alt1, lat2, long2, alt2, units="deg"):
    pos1 = navpy.lla2ecef(lat1, long1, alt1, latlon_unit=units)
    pos2 = navpy.lla2ecef(lat2, long2, alt2, latlon_unit=units)

    delta_pos = pos2 - pos1

    yaw = np.arctan2(delta_pos[1], delta_pos[0])
    pitch = np.arctan2(delta_pos[2] * np.cos(yaw), delta_pos[0])
    roll = np.arctan2(np.cos(yaw), np.sin(yaw) * np.sin(pitch))

    return yaw, pitch, roll
