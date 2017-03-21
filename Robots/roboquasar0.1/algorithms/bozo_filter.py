import math


class BozoFilter:
    def __init__(self, initial_compass):
        # angle offset variables
        self.compass_angle = None
        self.compass_angle_packet = "0.0"
        self.start_angle = None
        self.initialized = False

        if initial_compass is not None:
            self.init_compass(initial_compass)

        # bozo filter data
        self.lat_data = []
        self.long_data = []
        self.bearing = None
        self.imu_angle = None

        # bozo filter weights
        self.fast_rotation_threshold = 0.2
        self.bearing_avg_len = 4
        self.imu_angle_weight = 0.65

    def init_compass(self, packet):
        self.initialized = True
        self.compass_angle_packet = packet
        self.compass_angle = math.radians(float(packet)) - math.pi / 2
        print("initial offset: %0.4f rad" % self.compass_angle)

    def offset_angle(self, imu_euler_z):
        if self.start_angle is None:
            self.start_angle = imu_euler_z

        self.imu_angle = (imu_euler_z - self.start_angle + self.compass_angle) % (2 * math.pi)

        return self.imu_angle
        # if self.bearing is None or abs(self.imu.gyro.z) > self.fast_rotation_threshold:
        #     return -self.imu_angle, -self.imu_angle
        # else:
        #     if self.bearing - self.imu_angle > math.pi:
        #         self.imu_angle += 2 * math.pi
        #     if self.imu_angle - self.bearing > math.pi:
        #         self.bearing += 2 * math.pi
        #
        #     angle = self.imu_angle_weight * self.imu_angle + (1 - self.imu_angle_weight) * self.bearing
        #     return -angle

    def update_bearing(self, lat, long):
        if len(self.long_data) == 0 or long != self.long_data[-1]:
            self.long_data.append(long)
        if len(self.lat_data) == 0 or lat != self.lat_data[-1]:
            self.lat_data.append(lat)

        self.bearing = -math.atan2(long - self.long_data[0],
                                   lat - self.lat_data[0]) + math.pi
        self.bearing = (self.bearing - math.pi / 2) % (2 * math.pi)

        if len(self.long_data) > self.bearing_avg_len:
            self.long_data.pop(0)
        if len(self.lat_data) > self.bearing_avg_len:
            self.lat_data.pop(0)
