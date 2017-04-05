import math
import numpy as np

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
        if packet is not None:
            self.initialized = True
            if type(packet) == str:
                self.compass_angle_packet = packet
                self.compass_angle = math.radians(float(packet))
            else:  # assume float in radians
                self.compass_angle_packet = str(math.degrees(packet))
                self.compass_angle = packet

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

    def update_bearing(self, lat, lon):
        if len(self.long_data) == 0 or lon != self.long_data[-1]:
            self.long_data.append(lon)
        if len(self.lat_data) == 0 or lat != self.lat_data[-1]:
            self.lat_data.append(lat)

        self.bearing = -math.atan2(lon - self.long_data[0],
                                   lat - self.lat_data[0])# + math.pi
        self.bearing = self.bearing % (2 * math.pi)

        if len(self.long_data) > self.bearing_avg_len:
            self.long_data.pop(0)
        if len(self.lat_data) > self.bearing_avg_len:
            self.lat_data.pop(0)

class Geodesy:
    earth_radius = 6.371 * 10**6
    def __init__(self, initial_lon, initial_lat):
        self.lat_data = [initial_lat]
        self.lon_data = [initial_lon]

    def convert_radians(self, lat_lon):
        return lat_lon * np.pi/180

    def convert_degrees(self, lat_lon):
        return lat_lon * 180/np.pi

    def distance_to(self, lat0, lon0, lat1, lon1):
        lat0_r = self.convert_radians(lat0)
        lon0_r = self.convert_radians(lon0)
        lat1_r = self.convert_radians(lat1)
        lon1_r = self.convert_radians(lon1)
        delta_lat = lat1_r - lat0_r
        delta_lon = lon1_r - lon1_r
        a = (np.sin(delta_lat/2)**2 + 
            np.cos(lat1_r)*np.cos(lat0_r)*np.sin(delta_lon/2)**2)
        c = 2 * np.atan2(np.sqrt(a), np.sqrt(1-a))

        return  Geodesy.earth_radius * c

    def bearing_to(self, lat0, lon0, lat1, lon1):
        lat0_r = self.convert_radians(lat0)
        lon0_r = self.convert_radians(lon0)
        lat1_r = self.convert_radians(lat1)
        lon1_r = self.convert_radians(lon1)
        delta_lon = lon1_r - lon1_r
        y = np.sin(delta_lon) * np.cos(lat1_r)
        x = (np.cos(lat0_r)*np.sin(lat1_r) -
            np.sin(lat0_r)*np.cos(lat1_r)*np.cos(delta_lon))
        theta = Math.atan2(y, x)
        return (self.convert_degrees(theta)+360) %360
        final_bearing_to(self, lon0, lat0, lon1, lat1)
        return (self.bearing_to(lon1, lat1, lon0, lat0) + 180) % 360

    def midpoint_to(self, lat0, lon0, lat1, lon1):
        lat0_r = self.convert_radians(lat0)
        lon0_r = self.convert_radians(lon0)
        lat1_r = self.convert_radians(lat1)
        lon1_r = self.convert_radians(lon1)
        delta_lon = lon1_r - lon1_r
        Bx = np.cos(lat1_r)*np.cos(delta_lon)
        By = np.cos(lat1_r)*np.sin(delta_lon)
        x = np.sqrt((np.cos(lat0_r) + Bx)**2 + By**2)
        y = np.sin(lat0_r) + np.sin(lat1_r)
        mid_lat = np.atan2(y,x)
        mid_lon = lon0_r + np.atan2(By, np.cos(lat0_r) + Bx)
        return (self.convert_degrees(mid_lat), 
               (self.convert_radians(mid_lon) + 540)%360 -180);

    def intermediate_point(self, lat0, lon0, lat1, lon1, fraction):
        pass#implement?

    def destination_point(self, lat0, lon0, distance, bearing):
        ang_distance = distance/Geodesy.earth_radius
        lat0_r = self.convert_radians(lat0)
        lon0_r = self.convert_radians(lon0)
        lat1_r = (np.asin(np.sin(lat0_r)*np.cos(ang_distance) +
                         np.cos(lat_0r)*np.sin(ang_distance)*np.cos(bearing)))
        y = np.sin(bearing)*np.sin(ang_distance)*np.cos(lat0_r)
        x = np.cos(ang_distance) - np.sin(lat0_r) * np.sin(lat1_r)
        lon1_r = lon0_r + np.atan2(y,x)
        return (self.convert_degrees(lat1_r), 
                (self.convert_degrees(lon1_r) + 540)%360 -180)