"""
This class converts the sensor data into correct units
"""
import math


def almost_equal(value1, value2, epsilon):
    return abs(value1 - value2) <= epsilon


class Converter:
    def __init__(self, origin_lng, origin_lat, gps_epsilon,
                 wheel_diameter=0.271):
        self.origin_lng = origin_lng
        self.origin_lat = origin_lat

        self.prev_lng = origin_lng
        self.prev_lat = origin_lat

        self.prev_bind_x = 0
        self.prev_bind_y = 0

        self.prev_encoder = 0
        self.prev_gps_heading = 0
        self.prev_bind_heading = 0

        self.gps_epsilon = gps_epsilon
        self.wheel_diameter = wheel_diameter
        self.earth_radius = 6371000  # meters

    def convert_gps(self, prev_long, prev_lat, longitude, latitude):
        # lat_mean = (latitude + prev_lat) / 2
        # dlng = longitude - prev_long
        # dlat = latitude - prev_lat
        # x = dlng * math.cos(lat_mean) * self.deg_to_m
        # y = dlat * self.deg_to_m

        phi1 = math.radians(prev_lat)
        phi2 = math.radians(latitude)

        lam1 = math.radians(prev_long)
        lam2 = math.radians(longitude)

        dphi = math.radians(latitude - prev_lat)
        dlam = math.radians(longitude - prev_long)

        a = math.sin(dphi / 2) ** 2 + \
            math.cos(phi1) * math.cos(phi2) * \
            math.sin(dlam / 2) * math.sin(dlam / 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        dist = self.earth_radius * c

        y = math.sin(lam2 - lam1) * math.cos(phi2)
        x = math.cos(phi1) * math.sin(phi2) - \
            math.sin(phi1) * math.cos(phi2) * math.cos(lam2 - lam1)
        bearing = math.atan2(y, x)

        return dist * math.cos(bearing), dist * math.sin(bearing), bearing

    def convert_encoder(self, counts):
        delta_counts = counts - self.prev_encoder
        self.prev_encoder = counts
        return delta_counts * self.wheel_diameter * math.pi

    def convert_heading(self, longitude, latitude, bind_x, bind_y):
        """
        This converter takes the initial sensor data and makes it usable for the
        heading filter.

        In total, it converts lat, long into gps_heading,.
                  bound_x, bound_y into bound_heading

        NOTE: on first call, this interpreter will not have any data to
        derive a heading from. Therefore, it will return a 0 for both of
        these measurements which will not matter after 2 or 3 iterations
        (approx 1 second or less)
        """

        dx, dy, gps_heading = self.convert_gps(self.prev_lng, self.prev_lat, longitude,
                                  latitude)

        # gps_heading = math.atan2(dy, dx)

        if (almost_equal(self.prev_lng, longitude, self.gps_epsilon) and
                almost_equal(self.prev_lat, latitude, self.gps_epsilon)):
            gps_heading = self.prev_gps_heading
        else:
            self.prev_gps_heading = gps_heading

        dx = bind_x - self.prev_bind_x
        dy = bind_y - self.prev_bind_y

        bind_heading = math.atan2(dy, dx)

        if dx == 0 and dy == 0:
            bind_heading = self.prev_bind_heading

        else:
            self.prev_bind_heading = bind_heading

        self.prev_lat = latitude
        self.prev_lng = longitude

        self.prev_bind_x = bind_x
        self.prev_bind_y = bind_y

        return gps_heading, bind_heading

    def convert_position(self, longitude, latitude, encoder_counts):
        """
        This class converts the sensor data using the filtered heading to fit
        into cardinal direction conventions, where x is positive in the east
        direction and y is positive in the north direction.
        """

        x, y, bearing = self.convert_gps(self.origin_lng, self.origin_lat, longitude,
                                latitude)
        enc_dist = self.convert_encoder(encoder_counts)

        return x, y, enc_dist
