"""
This class converts the sensor data into correct units
"""
import math


class HeadingConverter:
    """
    This converter takes the initial sensor data and makes it usable for the heading filter.

    In total, it converts lat, long into gps_heading,.
              bound_x, bound_y into bound_heading

    NOTE: on first call, this interpreter will not have any data to
    derive a heading from. Therefore, it will return a 0 for both of
    these measurements which will not matter after 2 or 3 iterations
    (approx 1 second or less)
    """

    def __init__(self, initial_lng, initial_lat):
        self.prev_lat = initial_lat
        self.prev_lng = initial_lng
        self.prev_bind_x = 0
        self.prev_bind_y = 0

    def convert(self, longitude, latitude, bind_x, bind_y):
        dlong = longitude - self.prev_lng
        dlat = latitude - self.prev_lat

        gps_heading = math.atan2(dlat, dlong)

        dx = bind_x - self.prev_bind_x
        dy = bind_y - self.prev_bind_y

        bind_heading = math.atan2(dy, dx)

        self.prev_lat = latitude
        self.prev_lng = longitude

        self.prev_bind_x = bind_x
        self.prev_bind_y = bind_y

        return gps_heading, bind_heading


class PositionConverter:
    """
    This class converts the sensor data using the filtered heading to fit into
    cardinal direction conventions, where x is positive in the east direction and y
    is positive in the north direction
    """

    def __init__(self, initial_encoder, origin_lng, origin_lat):
        self.prev_encoder = initial_encoder
        self.origin_lng = origin_lng
        self.origin_lat = origin_lat
        self.wheel_diameter = 0.271  # in meters
        self.deg_to_m = 111226.343

    def convert(self, longitude, latitude, ax, ay,
                encoder_counts, heading):
        x, y = self.convert_gps(longitude, latitude)

        enc_dist = self.convert_encoder(encoder_counts)

        shifted_ax = (ax * math.cos(heading) - ay * math.sin(heading))

        shifted_ay = (ax * math.sin(heading) + ay * math.cos(heading))

        return x, y, shifted_ax, shifted_ay, enc_dist

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
