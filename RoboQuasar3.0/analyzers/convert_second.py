"""
This class converts the sensor data using the filtered heading to fit into
cardinal direction conventions, where x is positive in the east direction and y
is positive in the north direction
"""

import math


class Converter_position():
    def __init__(self, initial_encoder, origin_lng, origin_lat):
        self.prev_encoder = initial_encoder
        self.origin_lng = origin_lng
        self.origin_lat = origin_lat
        self.wheel_diameter = 0.271
        self.deg_to_m = 111226.343

    def convert(self, longitude, latitude, ax, ay,
                encoder_counts, heading):
        x, y = self.convert_gps(longitude, latitude)

        enc_dist = self.convert_encoder(encoder_counts)

        new_ax = (ax * math.cos(heading) -
                  ay * math.sin(heading))

        new_ay = (ax * math.sin(heading) +
                  ay * math.cos(heading))

        return (x, y, new_ax, new_ay, enc_dist)

    def convert_gps(self, longitude, latitude):
        lat_mean = (latitude + self.origin_lat) / 2
        dlng = longitude - self.origin_lng
        dlat = latitude - self.origin_lat
        x = dlng * math.cos(lat_mean) * self.deg_to_m
        y = dlat * self.deg_to_m

        return x, y

    def convert_encoder(self, counts):
        delta_counts = counts - self.prev_encoder
        self.prev_encoder = counts
        return delta_counts * self.wheel_diameter * math.pi
