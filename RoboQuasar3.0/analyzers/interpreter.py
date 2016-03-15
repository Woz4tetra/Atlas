"""
This class converts the sensor data into correct units
"""
import math

class Interpreter():
    def __init__(self, initial_encoder, origin_lat, origin_long, shift_angle):
        self.prev_encoder = initial_encoder
        self.origin_lat = origin_lat * math.pi / 180
        self.origin_long = origin_long * math.pi / 180
        self.wheel_radius = 0.1333  # TODO calculate wheel radius
        self.deg_to_m = 111226.343  # convert gps degrees to meters
        self.shift_angle = shift_angle

    def convert(self, latitude, longitude, enc_counts, yaw):
        x, y = self.convert_gps(latitude, longitude)
        enc_counts = self.convert_encoder(enc_counts)
        yaw = self.convert_imu(yaw)
        return x, y, enc_counts, yaw

    def convert_encoder(self, counts):
        delta_counts = counts - self.prev_encoder
        self.prev_encoder = counts
        return 2 * math.pi * self.wheel_radius * delta_counts

    def convert_gps(self, latitude, longitude):
        latitude *= math.pi / 180
        longitude *= math.pi / 180
        lat_mean = (latitude + self.origin_lat) / 2
        x = (longitude - self.origin_long) * math.cos(lat_mean) * self.deg_to_m
        y = (latitude - self.origin_lat) * self.deg_to_m

        shifted_x = x * math.cos(self.shift_angle) - y * math.sin(self.shift_angle)
        shifted_y = x * math.sin(self.shift_angle) + y * math.cos(self.shift_angle)
        return shifted_x, shifted_y

    def convert_imu(self, heading):
        return heading * math.pi / 180
