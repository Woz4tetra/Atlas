import math

import numpy as np

from analyzers.kalman_filter import KalmanFilter


class BuggyPiFilter:
    earth_radius = 6372797.6

    def __init__(self, initial_long, initial_lat, initial_heading,
                 counts_per_rotation, wheel_radius, front_back_dist):
        self.initial_long = math.radians(initial_long)
        self.initial_lat = math.radians(initial_lat)
        self.initial_heading = initial_heading  # assumed radians

        self.rotation = counts_per_rotation
        self.wheel_radius = wheel_radius
        self.front_back_dist = front_back_dist

        self.state_transition = np.zeros(6)
        self.control_matrix = np.eye(6)

        self.measurement_covariance = np.eye(6)
        process_error_covariance = np.eye(6)
        observation_matrix = np.eye(6)

        self.prev_gps_t = 0

        # treated as current gps coordinate until a new one comes in
        self.saved_gps_long = self.initial_long
        self.saved_gps_lat = self.initial_lat

        # the previous saved (or previous current) gps coordinate
        self.prev_gps_long = self.initial_long
        self.prev_gps_lat = self.initial_lat

        self.prev_time = 0

        self.prev_enc = 0
        self.enc_x_meters = 0
        self.enc_y_meters = 0
        self.prev_enc_long = None
        self.prev_enc_lat = None

        initial_state = np.array(
            [self.initial_lat, self.initial_long, self.initial_heading, 0, 0, 0]
        )
        initial_probability = np.eye(6)

        self.state = initial_state

        self.filter = KalmanFilter(initial_state, initial_probability,
                                   observation_matrix, process_error_covariance)

    def update_matrices(self, dt):
        self.state_transition[0][0] = dt
        self.state_transition[1][1] = dt
        self.state_transition[2][2] = dt

        self.control_matrix[0][0] = dt
        self.control_matrix[1][1] = dt
        self.control_matrix[2][2] = dt

    def update_measurement_covariance(self, timestamp):
        self.measurement_covariance[0][0] = (timestamp - self.prev_gps_t) * 1000
        self.measurement_covariance[1][1] = (timestamp - self.prev_gps_t) * 1000

    def update(self, enc_counts, imu_angular_yaw, speed_command,
               servo_value, timestamp, gps_x_long=None, gps_y_lat=None):
        dt = timestamp - self.prev_time
        self.update_matrices(dt)
        self.update_measurement_covariance(timestamp)

        if gps_x_long is not None and gps_y_lat is not None:
            self.prev_gps_t = timestamp + 1  # covariance should be 1 TODO: experiment with different curves
            self.prev_gps_long = self.saved_gps_long
            self.prev_gps_lat = self.saved_gps_lat

        self.update_measurement_covariance(timestamp)

        self.filter.update(
            self.get_control_vector(speed_command, servo_value),
            self.get_measurement(enc_counts, imu_angular_yaw, dt,
                                 gps_x_long, gps_y_lat),
            self.state_transition, self.control_matrix,
            self.measurement_covariance,
        )
        self.prev_time = timestamp

    def get_control_vector(self, speed, servo_value):
        servo_angle = self.servo_to_angle(servo_value)
        vx = speed * math.cos(servo_angle)
        vy = speed * math.sin(servo_angle)
        angular_v = speed * math.tan(servo_angle) / self.front_back_dist

        return np.array(
            [vx, vy, angular_v, vx, vy, angular_v]
        )

    def get_measurement(self, enc_counts, imu_angular_yaw, dt,
                        gps_x_long, gps_y_lat):
        if gps_x_long is None:
            gps_x_long = self.saved_gps_long
        if gps_y_lat is None:
            gps_y_lat = self.saved_gps_lat

        long_rad = math.radians(gps_x_long)
        lat_rad = math.radians(gps_y_lat)
        bearing = self.gps_bearing(
            long_rad, lat_rad, self.prev_gps_long, self.prev_gps_lat
        )

        enc_vx, enc_vy = self.get_enc_vel(enc_counts, dt)

        return np.array(
            [long_rad, lat_rad, bearing, enc_vx, enc_vy, imu_angular_yaw]
        )

    left_angle = 0.81096
    right_angle = -0.53719
    left_value = 35
    right_value = -25

    def servo_to_angle(self, servo_value):
        return ((self.left_angle - self.right_angle) /
                (self.left_value - self.right_angle) *
                (servo_value - self.right_angle) + self.right_angle)

    @staticmethod
    def gps_bearing(long, lat, prev_long, prev_lat):  # long & lat in radians
        y = math.sin(long - prev_long) * math.cos(lat)
        x = (math.cos(prev_lat) * math.sin(lat) - math.sin(
            prev_lat) * math.cos(lat) *
             math.cos(long - prev_long))
        bearing = math.atan2(y, x)

        return bearing

    def enc_to_meters(self, counts):
        return counts * self.wheel_radius / self.rotation * math.pi

    def get_enc_vel(self, enc_counts, dt):
        bearing = self.state[2]
        enc_speed = self.enc_to_meters(enc_counts - self.prev_enc) / dt
        enc_vx_meters = enc_speed * math.cos(bearing)
        enc_vy_meters = enc_speed * math.sin(bearing)

        self.enc_x_meters += enc_vx_meters
        self.enc_y_meters += enc_vy_meters

        enc_long, enc_lat = self.xy_meters_to_gps(
            self.enc_x_meters, self.enc_y_meters
        )

        vx = (enc_long - self.prev_enc_long) / dt
        vy = (enc_lat - self.prev_enc_lat) / dt

        self.prev_enc_long = enc_long
        self.prev_enc_lat = enc_lat

        return vx, vy

    def xy_meters_to_gps(self, x, y):
        dist = (x ** 2 + y ** 2) ** 0.5
        angle = math.atan2(y, x)
        return self.dist_to_gps(self.initial_long, self.initial_lat,
                                dist, angle)

    def dist_to_gps(self, initial_long, initial_lat, distance, bearing):
        lat = math.asin(
            math.sin(initial_lat) * math.cos(distance / self.earth_radius) +
            math.cos(initial_lat) * math.sin(distance / self.earth_radius) *
            math.cos(bearing))
        long = (initial_long +
                math.atan2(
                    math.sin(bearing) * math.sin(distance / self.earth_radius) *
                    math.cos(initial_lat),
                    math.cos(distance / self.earth_radius) - math.sin(
                        initial_lat)
                    * math.sin(lat)))
        return long, lat
