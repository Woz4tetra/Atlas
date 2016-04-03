"""
This class converts the sensor data into correct units
"""
import math


class Interpreter():
    def __init__(self, initial_encoder, origin_lat, origin_long):
        self.prev_encoder = initial_encoder
        self.origin_lat = origin_lat
        self.origin_long = origin_long
        self.wheel_radius = 0.1778  # wheel radius in meters
        self.deg_to_m = 111226.343  # convert gps degrees to meters

    def convert(self, latitude, longitude, accel_x, accel_y, enc_counts,
            kalman_heading):
        x, y = self.convert_gps(latitude, longitude)
        enc_dist = self.convert_encoder(enc_counts)

        shifted_accel_x = accel_x * math.cos(
            kalman_heading) - accel_y * math.sin(kalman_heading)
        shifted_accel_y = accel_x * math.sin(
            kalman_heading) + accel_y * math.cos(kalman_heading)

        return x, y, shifted_accel_x, shifted_accel_y, enc_dist

    def convert_encoder(self, counts):
        delta_counts = counts - self.prev_encoder
        self.prev_encoder = counts
        return 2 * math.pi * self.wheel_radius * delta_counts

    def convert_gps(self, latitude, longitude):
        lat_mean = (latitude + self.origin_lat) / 2
        x = (longitude - self.origin_long) * math.cos(lat_mean) * self.deg_to_m
        y = (latitude - self.origin_lat) * self.deg_to_m

        return x, y
