"""
This converter takes the initial sensor data and makes it usable for the heading filter.

In total, it converts lat, long into gps_heading,.
          bound_x, bound_y into bound_heading

NOTE: on first call, this interpreter will not have any data to
derive a heading from. Therefore, it will return a 0 for both of
these measurements which will not matter after 2 or 3 iterations
(approx 1 second or less)
"""

import math

class Converter_heading():

    def __init__(self, initial_lng, inital_lat):
        self.prev_lat = initial_lat
        self.prev_lng = initial_lng
        self.prev_bind_x  = 0
        self.prev_bind_y  = 0

    def convert(self, longitude, latitude, bind_x, bind_y):
        dlong = longitude - self.prev_lng
        dlat  = latitude  - self.prev_lat

        gps_heading = math.atan2(dlat, dlong)

        dx = bind_x - self.prev_bind_x
        dy = bind_y - self.prev_bind_y

        bind_heading = math.atan2(dy, dy)

        self.prev_lat = latitude
        self.prev_lng = longitude

        self.prev_bind_x = bind_x
        self.prev_bind_y = bind_y

        return gps_heading, bind_heading
