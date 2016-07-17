import math
from sys import maxsize as MAX_INT

import numpy as np

from navigation.kalman_filter import KalmanFilter


class BuggyPiFilter:
    # retrieved from http://www.geomidpoint.com/destination/
    earth_radius = 6372797.6

    def __init__(self, initial_long, initial_lat, initial_heading,
                 counts_per_rotation, wheel_radius, front_back_dist,
                 max_speed, left_angle_limit, right_angle_limit,
                 left_servo_limit, right_servo_limit):
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

        self.state_transition = np.eye(6)
        self.control_matrix = np.eye(6)

        # self.measurement_covariance = np.array([
        #     [1, 0, 0, 0, 0, 0],  # GPS long (will change during update)
        #     [0, 1, 0, 0, 0, 0],  # GPS lat (changes)
        #     [0, 0, 1, 0, 0, 0],  # GPS bearing (changes)
        #     [0, 0, 0, 1, 0, 0],  # encoder vx
        #     [0, 0, 0, 0, 1, 0],  # encoder vy
        #     [0, 0, 0, 0, 0, 1],  # imu angular velocity
        # ])

        self.process_error_covariance = np.array([
            [100, 0, 0, 0, 0, 0],
            [0, 100, 0, 0, 0, 0],
            [0, 0, 100000000, 0, 0, 0],
            [0, 0, 0, 100, 0, 0],
            [0, 0, 0, 0, 100, 0],
            [0, 0, 0, 0, 0, 1],
        ])

        self.observation_matrix = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1],
            [0, 0, 1, 0, 0, 0],
        ])
        self.measurement_covariance = np.eye(len(self.observation_matrix))

        self.initial_state = np.array(
            [self.initial_long_rad, self.initial_lat_rad, self.initial_heading,
             0.0, 0.0, 0.0]
        )
        self.initial_probability = np.eye(6)

        self._state = self.initial_state
        self.state = {}
        self.update_state_dict()

        self.filter = KalmanFilter(self.initial_state, self.state_transition,
                                   self.initial_probability,
                                   self.observation_matrix,
                                   self.process_error_covariance)

        # ----- unit conversion related variables -----

        self.left_angle = left_angle_limit
        self.right_angle = right_angle_limit
        self.left_value = left_servo_limit
        self.right_value = right_servo_limit

        self.prev_time = 0
        self.prev_enc_t = None
        self.prev_imu_t = None
        self.prev_gps_t = None
        self.prev_extrapolate_t = None

        self.enc_vx, self.enc_vy = 0.0, 0.0
        self.enc_speed = 0.0
        self.prev_enc = None

        self.imu_ang_v = 0.0
        self.start_imu = 0.0
        self.imu_yaw = 0.0
        self.prev_imu = None

        self.prev_gps_long = self.initial_long_rad
        self.prev_gps_lat = self.initial_lat_rad
        self.gps_x_meters, self.gps_y_meters = self.xy_meters_to_gps(
            self.prev_gps_long, self.prev_gps_lat
        )
        self.prev_gps_x_meters = 0.0
        self.prev_gps_y_meters = 0.0
        self.gps_bearing = 0.0  # 0.0 == east

        self.motor_speed = 0.0
        self.servo_angle = 0.0

    # ----- update sensors and commands -----

    def update_motors(self, motor_value):
        self.motor_speed = motor_value / 100 * self.max_speed

    def update_servo(self, servo_value):
        self.servo_angle = self.servo_to_angle(servo_value)

    def extrapolate_gps(self, timestamp):
        if self.prev_extrapolate_t is None:
            self.prev_extrapolate_t = timestamp
        else:
            dt = timestamp - self.prev_extrapolate_t
            self.gps_x_meters += self.state["vx"] * dt
            self.gps_y_meters += self.state["vy"] * dt

            self.gps_bearing += self.state["ang v"] * dt

            self.prev_extrapolate_t = timestamp

    def update_encoder(self, timestamp, enc_counts):
        if self.prev_enc_t is None:
            self.prev_enc_t = timestamp
            self.prev_enc = enc_counts

            return self.state
        else:
            dt = timestamp - self.prev_enc_t
            self.enc_vx, self.enc_vy, self.enc_speed = self.get_velocity(
                enc_counts, self.prev_enc, self.state["angle"], dt)

            self.prev_enc_t = timestamp
            self.prev_enc = enc_counts

            # self.extrapolate_gps(timestamp)
            self.update_covariances(timestamp)

            return self.update_filter(timestamp)

    def update_imu(self, timestamp, imu_yaw):
        if self.prev_imu_t is None:
            self.prev_imu_t = timestamp
            self.imu_yaw = imu_yaw
            self.start_imu = imu_yaw
            self.prev_imu = (imu_yaw - self.start_imu) + self.initial_heading

            return self.state
        else:
            dt = timestamp - self.prev_imu_t
            self.imu_yaw = (imu_yaw - self.start_imu) + self.initial_heading
            self.imu_ang_v = (self.imu_yaw - self.prev_imu) / dt
            # self.gps_bearing += imu_yaw - self.prev_imu

            self.prev_imu_t = timestamp
            self.prev_imu = self.imu_yaw

            # self.extrapolate_gps(timestamp)
            self.update_covariances(timestamp)

            return self.update_filter(timestamp)

    def update_gps(self, timestamp, gps_long, gps_lat):
        if self.prev_gps_t is None:
            self.prev_gps_t = timestamp
            if gps_long != self.prev_gps_long:
                self.prev_gps_long = gps_long
            if gps_lat != self.prev_gps_lat:
                self.prev_gps_lat = gps_lat

            return self.state
        else:
            self.gps_x_meters, self.gps_y_meters = \
                self.gps_to_xy_meters(gps_long, gps_lat)

            if gps_long != self.prev_gps_long or gps_lat != self.prev_gps_lat:
                self.gps_bearing = self.get_gps_bearing(
                    gps_long, gps_lat,
                    self.prev_gps_long, self.prev_gps_lat
                )
                self.gps_bearing = (-self.gps_bearing + math.pi / 2) % (2 * math.pi)

            self.prev_gps_t = timestamp
            self.update_covariances(timestamp)

            if gps_long != self.prev_gps_long:
                self.prev_gps_long = gps_long
            if gps_lat != self.prev_gps_lat:
                self.prev_gps_lat = gps_lat

            return self.update_filter(timestamp)

    # ----- update state and filter -----

    def update_filter(self, timestamp):
        # update control matrix with dt
        for index in range(0, 3):
            self.control_matrix[index][index] = timestamp - self.prev_time

        self._state = self.filter.update(
            self.get_command_vector(self.motor_speed, self.servo_angle),
            np.array([self.gps_x_meters, self.gps_y_meters, self.gps_bearing,
                      self.enc_vx, self.enc_vy, self.imu_ang_v, self.imu_yaw]),
            self.control_matrix, self.measurement_covariance,
        )

        self.prev_time = timestamp

        self.update_state_dict()
        return self.state

    def update_state_dict(self):
        long, lat = self.xy_meters_to_gps(self._state[0], self._state[1])
        self.state["x"] = math.degrees(long)  # gps long
        self.state["y"] = math.degrees(lat)  # gps lat
        self.state["angle"] = self._state[2]  # radians
        self.state["vx"] = self._state[3]  # meters / second
        self.state["vy"] = self._state[4]  # meters / second
        self.state["ang v"] = self._state[5]  # radians / second

    def update_covariances(self, timestamp):
        # covariance should be >= 1 TODO: experiment with different curves
        if self.prev_gps_t is not None:
            gps_covariance = int((math.exp(
                timestamp - self.prev_gps_t) - 1) * 1000000) + 1000000
            bearing_covariance = 10000 / (timestamp - self.prev_gps_t + 1) + 1#(timestamp - self.prev_gps_t) * 100000 + 10000000000

            if gps_covariance < MAX_INT:  # prevent overflow error
                self.measurement_covariance[0][0] = gps_covariance
                self.measurement_covariance[1][1] = gps_covariance
            if bearing_covariance < MAX_INT:
                self.measurement_covariance[2][2] = bearing_covariance

        if self.prev_enc_t is not None:
            if timestamp - self.prev_enc_t > 0.25:
                self.enc_vx = 0
                self.enc_vy = 0
            else:
                vel_covariance = (timestamp - self.prev_enc_t) * 1000 + 1

                if vel_covariance < MAX_INT:
                    self.measurement_covariance[3][3] = vel_covariance
                    self.measurement_covariance[4][4] = vel_covariance

        if self.prev_imu_t is not None:
            ang_v_covariance = (timestamp - self.prev_imu_t) + 1
            yaw_covariance = (timestamp - self.prev_imu_t) + 1
            if ang_v_covariance < MAX_INT:
                self.measurement_covariance[5][5] = ang_v_covariance
            if yaw_covariance < MAX_INT:
                self.measurement_covariance[6][6] = yaw_covariance

    # ----- conversion methods -----

    def get_command_vector(self, motor_speed, servo_angle):
        vx_command = motor_speed * math.cos(self._state[2] + servo_angle)
        vy_command = motor_speed * math.cos(self._state[2] + servo_angle)
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

        bearing = self.get_gps_bearing(long2, lat2, long1, lat1)

        x = dist * math.cos(bearing)
        y = dist * math.sin(bearing)
        return x, y

    def servo_to_angle(self, servo_value):
        return ((self.left_angle - self.right_angle) /
                (self.left_value - self.right_value) *
                (servo_value - self.right_value) + self.right_angle)

    @staticmethod
    def get_gps_bearing(long, lat, prev_long, prev_lat):
        long = math.radians(long)
        lat = math.radians(lat)
        prev_long = math.radians(prev_long)
        prev_lat = math.radians(prev_lat)

        y = math.sin(long - prev_long) * math.cos(lat)
        x = (math.cos(prev_lat) * math.sin(lat) - math.sin(
            prev_lat) * math.cos(lat) *
             math.cos(long - prev_long))

        return math.atan2(y, x)
