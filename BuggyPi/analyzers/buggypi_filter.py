import math

import numpy as np

from analyzers.kalman_filter import KalmanFilter


class BuggyPiFilter:
    earth_radius = 6372797.6

    def __init__(self, initial_long, initial_lat, initial_heading,
                 counts_per_rotation, wheel_radius, front_back_dist):
        # ----- filter related variables -----
        self.initial_long = initial_long
        self.initial_lat = initial_lat
        self.initial_heading = initial_heading  # assumed radians

        self.initial_long_rad = math.radians(initial_long)
        self.initial_lat_rad = math.radians(initial_lat)

        self.rotation = counts_per_rotation
        self.wheel_radius = wheel_radius
        self.front_back_dist = front_back_dist

        self.state_transition = np.eye(6)
        self.control_matrix = np.eye(6)

        self.measurement_covariance = np.eye(6)
        process_error_covariance = np.eye(6)
        observation_matrix = np.eye(6)

        self.prev_time = 0.0
        self.prev_gps_t = 0.0

        initial_state = np.array(
            [self.initial_long_rad, self.initial_lat_rad, self.initial_heading,
             0.0, 0.0, 0.0]
        )
        initial_probability = np.eye(6)

        self.state = initial_state

        self.filter = KalmanFilter(initial_state, initial_probability,
                                   observation_matrix, process_error_covariance)

        # ----- unit conversion related variables -----
        self.prev_enc = 0
        self.prev_enc_vx = 0
        self.prev_enc_vy = 0

        self.prev_imu = 0.0

        self.prev_gps_long = self.initial_long
        self.prev_gps_lat = self.initial_lat
        self.bearing = 0.0  # 0.0 == east
        self.preserved_gps_data = [0.0, 0.0]

    def update(self, timestamp, enc_counts, dt_enc,
               imu_yaw, dt_imu, servo_value,
               gps_long=None, gps_lat=None):
        vx, vy = self.get_speed(enc_counts, self.prev_enc, dt_enc)

        imu_angular_v = (imu_yaw - self.prev_imu) / dt_imu

        if gps_long is None or gps_lat is None:
            gps_long = self.preserved_gps_data[0]
            gps_lat = self.preserved_gps_data[1]
            gps_updated = False
        else:
            self.preserved_gps_data[0] = gps_long
            self.preserved_gps_data[1] = gps_lat
            gps_updated = True

        if gps_updated and (
                gps_long != self.prev_gps_long or gps_lat != self.prev_gps_lat):
            self.bearing = math.atan2(gps_lat - self.prev_gps_lat,
                                      gps_long - self.prev_gps_long)

        gps_x_meters, gps_y_meters = \
            self.gps_to_xy_meters(gps_long, gps_lat)

        if not gps_updated:
            self.bearing += imu_yaw - self.prev_imu

        servo_angle = self.servo_to_angle(servo_value)

        state = self.update_filter(
            timestamp, vx, vy, imu_angular_v,
            self.prev_enc_vx, self.prev_enc_vy,
            servo_angle, gps_x_meters, gps_y_meters,
            self.bearing, gps_updated)

        self.prev_enc = enc_counts
        self.prev_enc_vx = vx
        self.prev_enc_vy = vy

        self.prev_imu = imu_yaw

        self.prev_gps_long = gps_long
        self.prev_gps_lat = gps_lat

        return state

    def get_speed(self, counts, prev_count, dt):
        current_speed = self.enc_to_meters(counts - prev_count) / dt

        # using previous heading estimation
        vx = current_speed * math.cos(self.state[2])
        vy = current_speed * math.sin(self.state[2])
        return vx, vy

    def enc_to_meters(self, counts):
        return counts * self.wheel_radius / self.rotation * math.pi

    def update_filter(self, timestamp, enc_vx, enc_vy, imu_angular_yaw,
                      vx_command, vy_command, angle_command,
                      gps_x_meters, gps_y_meters, gps_bearing, gps_updated):
        dt = timestamp - self.prev_time

        # update control matrix with dt
        for index in range(0, 3):
            self.control_matrix[index][index] = dt

        # reset measurement covariances
        if gps_updated:
            self.prev_gps_t = timestamp

        # covariance should be >= 1 TODO: experiment with different curves
        self.measurement_covariance[0][0] = \
            (timestamp - self.prev_gps_t) * 100000 + 1
        self.measurement_covariance[1][1] = \
            (timestamp - self.prev_gps_t) * 100000 + 1

        self.state = self.filter.update(
            np.array([vx_command, vy_command, angle_command,
                      vx_command, vy_command, angle_command]),
            np.array([gps_x_meters, gps_y_meters, gps_bearing,
                      enc_vx, enc_vy, imu_angular_yaw]),
            self.state_transition, self.control_matrix,
            self.measurement_covariance,
        )
        self.prev_time = timestamp
        return self.get_state()

    def get_state(self):
        x, y, heading = self.state[0:3]
        long, lat = self.xy_meters_to_gps(x, y)
        return math.degrees(long), math.degrees(lat), heading

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

    def get_control_vector(self, speed, servo_value, heading):
        servo_angle = self.servo_to_angle(servo_value)
        vx = speed * math.cos(heading)
        vy = speed * math.sin(heading)
        angular_v = speed * math.tan(servo_angle) / self.front_back_dist

        return np.array(
            [vx, vy, angular_v, vx, vy, angular_v]
        )

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

        #
        # def get_enc_vel(self, enc_counts, enc_dt):
        #     heading = self.state[2]
        #     enc_dist = self.enc_to_meters(enc_counts - self.prev_enc)
        #     self.prev_enc = enc_counts
        #
        #     # self.enc_x_meters += enc_dist * math.cos(bearing)
        #     # self.enc_y_meters += enc_dist * math.sin(bearing)
        #
        #     # enc_long, enc_lat = self.xy_meters_to_gps(
        #     #     self.enc_x_meters, self.enc_y_meters
        #     # )
        #     # dt *= 10
        #     vx = enc_dist * math.cos(
        #         heading) / enc_dt
        #     vy = enc_dist * math.sin(heading) / enc_dt
        #     # self.prev_enc_long = enc_long
        #     # self.prev_enc_lat = enc_lat
        #
        #     return vx, vy
        #
