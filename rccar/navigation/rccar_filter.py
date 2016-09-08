"""An implementation of the kalman filter tailored for the BuggyPi rc car."""

import math
import numpy as np
from autobuggy.kalman_filter import KalmanFilter


class RcCarFilter:
    # retrieved from http://www.geomidpoint.com/destination/
    earth_radius = 6372797.6

    def __init__(self, counts_per_rotation, wheel_radius, front_back_dist,
                 max_speed, left_angle_limit, right_angle_limit,
                 left_servo_limit, right_servo_limit,
                 initial_long=None, initial_lat=None, initial_heading=None):
        """
        If initial conditions aren't provided, initialize_filter should be
        called later before the filter can be used.

        :param counts_per_rotation: Number of encoder counts per rotation
        :param wheel_radius: Radius of the wheel the encoder is on
        :param front_back_dist: Distance between the front and back axle
        :param max_speed: Maximum speed in meters/second of the robot
        :param left_angle_limit: Left steering limit in radians
        :param right_angle_limit: Right steering limit in radians
        :param left_servo_limit: Left steering limit in servo values
        :param right_servo_limit: Right steering limit in servo values
        :param initial_long: Initial longitude (x) of the robot in GPS degrees
        :param initial_lat: Initial latitude (y) of the robot in GPS degrees
        :param initial_heading: Initial direction of the robot in radians
        """

        # ----- filter related variables -----
        self.rotation = counts_per_rotation
        self.wheel_radius = wheel_radius
        self.front_back_dist = front_back_dist
        self.max_speed = max_speed

        self.left_angle_limit = left_angle_limit
        self.right_angle_limit = right_angle_limit
        self.left_servo_limit = left_servo_limit
        self.right_servo_limit = right_servo_limit

        if initial_long is not None or \
                        initial_lat is not None or \
                        initial_heading is not None:
            self.initialize_filter(initial_long, initial_lat, initial_heading)

    def initialize_filter(self, initial_long, initial_lat, initial_heading):
        """Initialize all filter matrices"""
        self.initial_long = initial_long
        self.initial_lat = initial_lat
        self.initial_heading = initial_heading  # assumed radians

        self.initial_long_rad = math.radians(initial_long)
        self.initial_lat_rad = math.radians(initial_lat)

        self.state_transition = np.eye(6)  # updated in update_state_transition
        self.control_matrix = np.eye(6)  # updated in update_filter

        # amount of error between each filter update
        self.process_error_covariance = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1],
        ])

        # Translates state vectors into the format of a measurement vector.
        # The rccar has one measurement for each value in the state vector,
        # so it's one to one
        self.observation_matrix = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1],
            # [0, 0, 1, 0, 0, 0],
            # [1, 0, 0, 0, 0, 0],
            # [0, 1, 0, 0, 0, 0],
        ])

        # a measurement vector:
        # [gps x, gps y, gps bearing, encoder vx, encoder vy, imu angular v]
        self.measurement = np.zeros(len(self.observation_matrix))

        # The error of each sensor
        self.measurement_covariance = np.eye(len(self.observation_matrix))
        self.measurement_covariance[0][0] = 10000  # gps long
        self.measurement_covariance[1][1] = 10000  # gps lat
        self.measurement_covariance[2][2] = 1  # gps bearing
        # self.measurement_covariance[3][3] = 1
        # self.measurement_covariance[4][4] = 1
        # self.measurement_covariance[5][5] = 1
        # self.measurement_covariance[6][6] = 1.5
        # self.measurement_covariance[7][7] = 1.5

        # state uses meters and always starts at 0, 0
        self.initial_state = np.array(
            [0.0, 0.0, self.initial_heading, 0.0, 0.0, 0.0]
        )
        self.initial_probability = np.eye(6)

        self._state = self.initial_state
        self.state = {}  # meters are converted to gps degrees
        self.update_state_dict()  # put state vector into dictionary

        self.filter = KalmanFilter(self.initial_state,
                                   self.initial_probability,
                                   self.process_error_covariance)
        # ----- unit conversion related variables -----

        # Use shorter names for servo limits
        self.left_angle = self.left_angle_limit  # radians
        self.right_angle = self.right_angle_limit  # radians
        self.left_value = self.left_servo_limit  # servo counts
        self.right_value = self.right_servo_limit  # servo counts


        self.prev_time = 0
        self.prev_enc_t = None
        self.prev_imu_t = None
        self.prev_gps_t = None
        self.prev_extrapolate_t = None

        self.enc_vx, self.enc_vy = 0.0, 0.0
        self.enc_x, self.enc_y = 0.0, 0.0
        self.prev_enc = None

        self.imu_ang_v = 0.0
        self.start_imu = 0.0
        self.imu_yaw = self.initial_heading
        self.prev_imu = self.initial_heading

        self.prev_gps_long = self.initial_long_rad
        self.prev_gps_lat = self.initial_lat_rad
        self.gps_x_meters, self.gps_y_meters = 0.0, 0.0
        self.prev_gps_x_meters, self.prev_gps_y_meters = 0.0, 0.0
        self.gps_bearing = 0.0  # 0.0 == east

        # how much to weigh the IMU when calculating GPS bearing
        self.imu_angle_weight = 0.5

        self.motor_speed = 0.0
        self.servo_angle = 0.0

        # ----- update sensors and commands -----

    def update_motors(self, motor_value):
        """
        Call this if the motor value changed. Must be a value from
        -100...100
        """
        self.motor_speed = motor_value / 100 * self.max_speed

    def update_servo(self, servo_value):
        """
        Call this if the servo value changed. Must be a value from
        -90...90
        """
        self.servo_angle = self.servo_to_angle(servo_value)

    def update_encoder(self, timestamp, enc_counts):
        """Call this if the encoder value changed"""
        if self.prev_enc_t is None:
            self.prev_enc_t = timestamp
            self.prev_enc = enc_counts

            # return self.state
        else:
            dt = timestamp - self.prev_enc_t
            self.enc_vx, self.enc_vy = self.get_velocity(
                enc_counts, self.prev_enc, self.gps_bearing, self.imu_yaw, dt,
                timestamp)
            self.enc_x += self.enc_vx * dt
            self.enc_y += self.enc_vy * dt

            self.prev_enc_t = timestamp
            self.prev_enc = enc_counts

            # self.update_covariances(timestamp)

            # return self.update_filter(timestamp)

    def adjust_yaw(self, imu_yaw):
        return ((imu_yaw - self.start_imu) + self.initial_heading) % (
            2 * math.pi)

    def update_imu(self, timestamp, imu_yaw):
        if imu_yaw is not None:
            if self.prev_imu_t is None:
                self.prev_imu_t = timestamp
                self.start_imu = imu_yaw
                # return self.state
            else:
                dt = timestamp - self.prev_imu_t
                self.imu_yaw = self.adjust_yaw(imu_yaw)
                self.imu_ang_v = (self.imu_yaw - self.prev_imu) / dt

                self.prev_imu_t = timestamp
                self.prev_imu = self.imu_yaw

                # self.update_covariances(timestamp)

                # return self.update_filter(timestamp)

    def update_gps(self, timestamp, gps_long, gps_lat):
        if self.prev_gps_t is None:
            self.prev_gps_t = timestamp
            if gps_long != self.prev_gps_long:
                self.prev_gps_long = gps_long
            if gps_lat != self.prev_gps_lat:
                self.prev_gps_lat = gps_lat

                # return self.state
        else:
            self.gps_x_meters, self.gps_y_meters = \
                self.gps_to_xy_meters(gps_long, gps_lat)

            if gps_long != self.prev_gps_long or gps_lat != self.prev_gps_lat:
                # self.gps_bearing = self.get_gps_bearing(
                #     gps_long, gps_lat,
                #     self.prev_gps_long, self.prev_gps_lat
                # )
                self.gps_bearing = self.my_atan2(
                    self.gps_y_meters - self.prev_gps_y_meters,
                    self.gps_x_meters - self.prev_gps_x_meters)

            self.prev_gps_t = timestamp
            # self.update_covariances(timestamp)

            if gps_long != self.prev_gps_long:
                self.prev_gps_long = gps_long
                self.prev_gps_x_meters = self.gps_x_meters
            if gps_lat != self.prev_gps_lat:
                self.prev_gps_lat = gps_lat
                self.prev_gps_y_meters = self.gps_y_meters

                # return self.update_filter(timestamp)

    # ----- update state and filter -----

    def update_filter(self, timestamp):
        # update control matrix with dt
        dt = timestamp - self.prev_time
        for index in range(0, 3):
            self.control_matrix[index][index] = dt

        if self.prev_enc_t is not None:
            # if encoders haven't updated for more than 0.25 seconds,
            # the robot's probably not moving.
            # Only assign it for a brief time window instead of over and over
            if 0.25 < timestamp - self.prev_enc_t < 0.3:
                self.enc_vx = 0
                self.enc_vy = 0

        self.update_state_transition(dt)

        self.measurement = np.array(
            [self.gps_x_meters, self.gps_y_meters, self.gps_bearing,
             self.enc_vx, self.enc_vy, self.imu_ang_v])
        self._state = self.filter.update(
            self.get_command_vector(self.motor_speed, self.servo_angle, dt),
            self.measurement,
            self.control_matrix, self.measurement_covariance,
            self.state_transition,
            self.observation_matrix
        )

        self.prev_time = timestamp

        self.update_state_dict()
        return self.state

    def update_all(self, timestamp, enc_counts, imu_yaw, gps_long, gps_lat):
        self.update_gps(timestamp, gps_long, gps_lat)
        self.update_imu(timestamp, imu_yaw)
        self.update_encoder(timestamp, enc_counts)
        return self.update_filter(timestamp)

    def update_state_dict(self):
        long, lat = self.xy_meters_to_gps(self._state[0], self._state[1])
        self.state["x"] = math.degrees(long)  # gps long
        self.state["y"] = math.degrees(lat)  # gps lat
        self.state["x m"] = self._state[0]  # gps long in meters (from start)
        self.state["y m"] = self._state[1]  # gps lat in meters (from start)
        self.state["angle"] = self._state[2]
        self.state["vx"] = self._state[3]  # meters / second
        self.state["vy"] = self._state[4]  # meters / second
        self.state["ang v"] = self._state[5]  # radians / second

        # def update_covariances(self, timestamp):
        #     pass
        # if self.prev_gps_t is not None:
        #     gps_covariance = \
        #         math.exp((timestamp - self.prev_gps_t) * 100) + 1
        #     bearing_covariance = \
        #         math.exp((timestamp - self.prev_gps_t * 50)) + 1
        #     print(gps_covariance)
        #     self.measurement_covariance[0][0] = gps_covariance
        #     self.measurement_covariance[1][1] = gps_covariance
        #     self.measurement_covariance[2][2] = bearing_covariance
        #
        # if self.prev_imu_t is not None:
        #     ang_v_covariance = (timestamp - self.prev_imu_t) + 1
        #     yaw_covariance = (timestamp - self.prev_imu_t) + 1
        #     if ang_v_covariance < MAX_INT:
        #         self.measurement_covariance[5][5] = ang_v_covariance
        #     if yaw_covariance < MAX_INT:
        #         self.measurement_covariance[6][6] = yaw_covariance

    def get_command_vector(self, motor_speed, servo_angle, dt):
        angle = self._state[2] + self._state[5] * dt
        vx_command = motor_speed * math.cos(angle)
        vy_command = motor_speed * math.sin(angle)
        ang_v_command = motor_speed * math.tan(
            servo_angle) / self.front_back_dist

        return np.array([vx_command, vy_command, ang_v_command,
                         vx_command, vy_command, ang_v_command])

    def update_state_transition(self, dt):
        self.state_transition[0][3] = dt  # x0 + vx0 * dt = x1
        self.state_transition[1][4] = dt  # y0 + vy0 * dt = y1
        self.state_transition[2][5] = dt  # theta0 + ang_v0 * dt = theta1

        # ----- conversion methods -----

    def get_velocity(self, counts, prev_count, gps_bearing, imu_yaw, dt,
                     timestamp):
        current_speed = self.enc_to_meters(counts - prev_count) / dt
        if current_speed > 0.2:  # and timestamp > 10:
            heading = (gps_bearing * (
                1 - self.imu_angle_weight) + imu_yaw * self.imu_angle_weight) % (
                          2 * math.pi)
        else:
            heading = imu_yaw
        vx = current_speed * math.cos(heading)
        vy = current_speed * math.sin(heading)
        return vx, vy

    def enc_to_meters(self, counts):
        return counts * self.wheel_radius / self.rotation * math.pi

    def xy_meters_to_gps(self, x, y):
        # from empirical testing, this is the fastest method of
        # calculating distance
        dist = ((x * x) + (y * y)) ** 0.5
        angle = -self.my_atan2(y, x) + math.pi / 2
        return self.dist_to_gps(dist, angle)

    def dist_to_gps(self, distance, bearing):
        lat = math.asin(
            math.sin(self.initial_lat_rad) * math.cos(
                distance / self.earth_radius) +
            math.cos(self.initial_lat_rad) * math.sin(
                distance / self.earth_radius) *
            math.cos(bearing))
        long = (self.initial_long_rad +
                math.atan2(
                    math.sin(bearing) * math.sin(distance / self.earth_radius) *
                    math.cos(self.initial_lat_rad),
                    math.cos(distance / self.earth_radius) - math.sin(
                        self.initial_lat_rad)
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

        bearing = self.get_gps_bearing(
            long2, lat2, long1, lat1, my_convention=False,
            convert_to_radians=False)

        # my convention has 0 at east not north. Swapped on purpose
        y = dist * math.cos(bearing)
        x = dist * math.sin(bearing)
        return x, y

    def servo_to_angle(self, servo_value):
        return ((self.left_angle - self.right_angle) /
                (self.left_value - self.right_value) *
                (servo_value - self.right_value) + self.right_angle)

    @staticmethod
    def my_atan2(y, x):
        angle = math.atan2(y, x)
        if angle < 0:
            angle += 2 * math.pi
        return angle

    @staticmethod
    def get_gps_bearing(long, lat, prev_long, prev_lat, my_convention=True,
                        convert_to_radians=True):
        if convert_to_radians:
            long = math.radians(long)
            lat = math.radians(lat)
            prev_long = math.radians(prev_long)
            prev_lat = math.radians(prev_lat)

        if my_convention:
            return RcCarFilter.my_atan2(lat - prev_lat, long - prev_long)
        else:
            y = math.sin(long - prev_long) * math.cos(lat)
            x = (math.cos(prev_lat) * math.sin(lat) - math.sin(
                prev_lat) * math.cos(lat) * math.cos(long - prev_long))

            return RcCarFilter.my_atan2(y, x)
