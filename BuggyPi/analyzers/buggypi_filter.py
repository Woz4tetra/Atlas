import math
from sys import maxsize as MAX_INT

import numpy as np

from analyzers.kalman_filter import KalmanFilter


class BuggyPiFilter:
    # retrieved from http://www.geomidpoint.com/destination/
    earth_radius = 6372797.6

    def __init__(self, initial_long, initial_lat, initial_heading,
                 counts_per_rotation, wheel_radius, front_back_dist,
                 max_speed):
        # ----- filter related variables -----
        self.initial_long = initial_long
        self.initial_lat = initial_lat
        self.initial_heading = initial_heading  # assumed radians

        self.initial_long_rad = math.radians(initial_long)
        self.initial_lat_rad = math.radians(initial_lat)

        self.rotation = counts_per_rotation
        self.wheel_radius = wheel_radius
        self.front_back_dist = front_back_dist
        self.max_speed = max_speed

        state_transition = np.eye(6)
        self.control_matrix = np.eye(6)

        self.measurement_covariance = np.array([
            [1, 0, 0, 0, 0, 0],  # GPS long (will change during update)
            [0, 1, 0, 0, 0, 0],  # GPS lat (changes)
            [0, 0, 1, 0, 0, 0],  # GPS bearing (changes)
            [0, 0, 0, 1, 0, 0],  # encoder vx
            [0, 0, 0, 0, 1, 0],  # encoder vy
            [0, 0, 0, 0, 0, 1],  # imu angular velocity
        ])

        process_error_covariance = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1],
        ])

        observation_matrix = np.eye(6)

        initial_state = np.array(
            [self.initial_long_rad, self.initial_lat_rad, self.initial_heading,
             0.0, 0.0, 0.0]
        )
        initial_probability = np.eye(6)

        self._state = initial_state
        self.state = {}
        self.update_state_dict()

        self.filter = KalmanFilter(initial_state, state_transition,
                                   initial_probability, observation_matrix,
                                   process_error_covariance)

        # ----- unit conversion related variables -----
        self.prev_time = 0.0
        self.prev_enc_t = 0.0
        self.prev_imu_t = 0.0
        self.prev_gps_t = 0.0
        self.prev_extrapolate_t = 0.0

        self.enc_vx, self.enc_vy = 0.0, 0.0
        self.enc_speed = 0.0
        self.prev_enc = 0

        self.imu_ang_v = 0.0
        self.prev_imu = 0

        self.prev_gps_long = self.initial_long_rad
        self.prev_gps_lat = self.initial_lat_rad
        self.gps_long = self.initial_long_rad
        self.gps_lat = self.initial_lat_rad
        self.gps_x_meters = 0.0
        self.gps_y_meters = 0.0
        self.bearing = 0.0  # 0.0 == east

        self.motor_speed = 0.0
        self.servo_angle = 0.0

    # ----- update sensors and commands -----

    def update_motors(self, motor_value):
        self.motor_speed = motor_value / 100 * self.max_speed

    def update_servo(self, servo_value):
        self.servo_angle = self.servo_to_angle(servo_value)

    def extrapolate_gps(self, timestamp):
        dt = timestamp - self.prev_extrapolate_t
        self.gps_x_meters += self.state["vx"] * dt
        self.gps_y_meters += self.state["vy"] * dt

        self.bearing += self.state["ang v"] * dt

        self.prev_extrapolate_t = timestamp

    def update_encoder(self, timestamp, enc_counts):
        dt = timestamp - self.prev_enc_t
        self.enc_vx, self.enc_vy, self.enc_speed = self.get_velocity(
            enc_counts, self.prev_enc, self._state[2], dt)
        self.prev_enc_t = timestamp
        self.prev_enc = enc_counts

        self.extrapolate_gps(timestamp)
        self.update_covariances(timestamp)

        return self.update_filter(
            timestamp, self.enc_vx, self.enc_vy, self.imu_ang_v,
            self.motor_speed, self.servo_angle, self.gps_x_meters,
            self.gps_y_meters, self.bearing
        )

    def update_imu(self, timestamp, imu_yaw):
        dt = timestamp - self.prev_imu_t
        self.imu_ang_v = (imu_yaw - self.prev_imu) / dt
        self.prev_imu_t = timestamp
        self.prev_imu = imu_yaw

        self.extrapolate_gps(timestamp)
        self.update_covariances(timestamp)

        return self.update_filter(
            timestamp, self.enc_vx, self.enc_vy, self.imu_ang_v,
            self.motor_speed, self.servo_angle, self.gps_x_meters,
            self.gps_y_meters, self.bearing
        )

    def update_gps(self, timestamp, gps_long, gps_lat):
        self.gps_x_meters, self.gps_y_meters = \
            self.gps_to_xy_meters(gps_long, gps_lat)

        self.bearing = math.atan2(gps_lat - self.prev_gps_lat,
                                  gps_long - self.prev_gps_long)

        self.prev_gps_t = timestamp
        self.update_covariances(timestamp)

        self.prev_gps_long = gps_long
        self.prev_gps_lat = gps_lat

        return self.update_filter(
            timestamp, self.enc_vx, self.enc_vy, self.imu_ang_v,
            self.motor_speed, self.servo_angle, self.gps_x_meters,
            self.gps_y_meters, self.bearing
        )

    # ----- update state and filter -----

    def update_filter(self, timestamp, enc_vx, enc_vy, imu_angular_yaw,
                      motor_speed, servo_angle,
                      gps_x_meters, gps_y_meters, gps_bearing):
        dt = timestamp - self.prev_time

        # update control matrix with dt
        for index in range(0, 3):
            self.control_matrix[index][index] = dt

        self._state = self.filter.update(
            self.get_command_vector(motor_speed, servo_angle),
            np.array([gps_x_meters, gps_y_meters, gps_bearing,
                      enc_vx, enc_vy, imu_angular_yaw]),
            self.control_matrix, self.measurement_covariance,
        )

        self.prev_time = timestamp

        self.update_state_dict()
        return self.state

    def update_state_dict(self):
        x, y, heading = self._state[0:3]
        long, lat = self.xy_meters_to_gps(x, y)
        self.state["x"] = math.degrees(long)  # gps long
        self.state["y"] = math.degrees(lat)   # gps lat
        self.state["angle"] = heading         # radians
        self.state["vx"] = self._state[3]     # meters / second
        self.state["vy"] = self._state[4]     # meters / second
        self.state["ang v"] = self._state[5]  # radians / second

    def update_covariances(self, timestamp):
        # covariance should be >= 1 TODO: experiment with different curves
        gps_covariance = int((math.exp(
            timestamp - self.prev_gps_t) - 1) * 10000) + 1
        bearing_covariance = (timestamp - self.prev_gps_t) * 10000 + 10000
        vel_covariance = (timestamp - self.prev_enc_t) * 1000 + 1
        ang_v_covariance = (timestamp - self.prev_imu_t) * 100 + 1

        if gps_covariance < MAX_INT:  # prevent overflow error
            self.measurement_covariance[0][0] = gps_covariance
            self.measurement_covariance[1][1] = gps_covariance
        if bearing_covariance < MAX_INT:
            self.measurement_covariance[2][2] = bearing_covariance
        if vel_covariance < MAX_INT:
            self.measurement_covariance[3][3] = vel_covariance
            self.measurement_covariance[4][4] = vel_covariance
        if ang_v_covariance < MAX_INT:
            self.measurement_covariance[5][5] = ang_v_covariance

    # ----- conversion methods -----

    def get_command_vector(self, motor_speed, servo_angle):
        vx_command = motor_speed * math.cos(self._state[2])
        vy_command = motor_speed * math.cos(self._state[2])
        ang_v_command = motor_speed * math.tan(
            servo_angle) / self.front_back_dist

        return np.array([vx_command, vy_command, ang_v_command,
                         vx_command, vy_command, ang_v_command])

    def get_velocity(self, counts, prev_count, heading, dt):
        current_speed = self.enc_to_meters(counts - prev_count) / dt

        vx = current_speed * math.cos(heading)
        vy = current_speed * math.sin(heading)
        return vx, vy, current_speed

    def enc_to_meters(self, counts):
        return counts * self.wheel_radius / self.rotation * math.pi

    def xy_meters_to_gps(self, x, y):
        dist = (x ** 2 + y ** 2) ** 0.5
        angle = math.atan2(y, x)
        return self.dist_to_gps(self.initial_long_rad, self.initial_lat_rad,
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

    def gps_to_xy_meters(self, gps_x_long, gps_y_lat):
        long2 = math.radians(gps_x_long)
        lat2 = math.radians(gps_y_lat)
        long1 = self.initial_long_rad
        lat1 = self.initial_lat_rad
        d_long = long2 - long1
        d_lat = lat2 - lat1

        a = (math.sin(d_lat / 2) * math.sin(d_lat / 2) +
             math.cos(lat1) * math.cos(lat2) *
             math.sin(d_long / 2) * math.sin(d_long / 2))
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        dist = self.earth_radius * c

        bearing = self.gps_bearing(long2, lat2, long1, lat1)

        x = dist * math.cos(bearing)
        y = dist * math.sin(bearing)
        return x, y

    left_angle = 0.81096
    right_angle = -0.53719
    left_value = 35
    right_value = -25

    def servo_to_angle(self, servo_value):
        return ((self.left_angle - self.right_angle) /
                (self.left_value - self.right_value) *
                (servo_value - self.right_value) + self.right_angle)

    @staticmethod
    def gps_bearing(long, lat, prev_long, prev_lat):  # long & lat in radians
        long = math.radians(long)
        lat = math.radians(lat)
        prev_long = math.radians(prev_long)
        prev_lat = math.radians(prev_lat)

        y = math.sin(long - prev_long) * math.cos(lat)
        x = (math.cos(prev_lat) * math.sin(lat) - math.sin(
            prev_lat) * math.cos(lat) *
             math.cos(long - prev_long))
        bearing = math.atan2(y, x)

        return bearing
