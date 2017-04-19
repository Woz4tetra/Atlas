import math
import sys

from atlasbuggy.files.mapfile import MapFile


class MapManipulator:
    MAX_FLOAT = sys.float_info.max
    MIN_FLOAT = sys.float_info.min

    def __init__(self, course_map_name, map_dir, inner_map_name=None, outer_map_name=None, offset=1,
                 border_skip_check=10, keep_position_in_boundary=False, init_indices=None):
        self.init_maps(course_map_name, map_dir, inner_map_name, outer_map_name)

        self.offset = offset
        self.border_skip_check = border_skip_check
        self.current_index = None
        self.current_angle = 0.0
        self.current_pos = self.map[0]
        self.goal_index = 0
        self.goal_angle = 0.0
        self.keep_position_in_boundary = keep_position_in_boundary
        self.goal_lat = 0.0
        self.goal_long = 0.0
        self.init_indices = init_indices

    def init_maps(self, course_map_name, map_dir, inner_map_name=None, outer_map_name=None):
        self.course_map_name = course_map_name
        self.inner_map_name = inner_map_name
        self.outer_map_name = outer_map_name
        self.map_dir = map_dir

        self.map = MapFile(self.course_map_name, map_dir)
        if self.inner_map_name is None:
            self.inner_map = None
        else:
            self.inner_map = MapFile(self.inner_map_name, map_dir)
        if self.outer_map_name is None:
            self.outer_map = None
        else:
            self.outer_map = MapFile(self.outer_map_name, map_dir)

    def initialize(self, lat, long):
        return self.get_goal(lat, long)

    def is_initialized(self):
        return self.current_index is not None

    def update(self, lat, long, yaw, offset=None, goal_num=None):
        self.goal_angle = self.get_goal_angle(lat, long, offset, goal_num)
        if self.goal_angle < 0:
            self.goal_angle += 2 * math.pi

        angle_error = self.shift_angle(self.goal_angle - yaw)

        self.current_angle = angle_error
        return self.current_angle

    def get_goal_angle(self, lat, long, offset=None, goal_num=None):
        goal_index = self.get_goal(lat, long, offset, goal_num)
        goal_lat, goal_long = self.map[goal_index]

        return math.atan2(goal_long - long, goal_lat - lat)

    def get_goal(self, lat0, long0, offset=None, goal_num=None):
        if self.keep_position_in_boundary:
            if not self.point_inside_outer_map(lat0, long0):
                nearest_index = self.closest_point(lat0, long0, self.outer_map)
                lat0, long0 = self.outer_map[nearest_index]
            if not self.point_outside_inner_map(lat0, long0):
                nearest_index = self.closest_point(lat0, long0, self.inner_map)
                lat0, long0 = self.inner_map[nearest_index]

        self.current_pos = lat0, long0

        # if self.current_index is None:
        start = 0
        end = len(self.map)
        # else:
        #     start = (self.goal_index - self.offset) % len(self.map)
        #     end = (start + self.offset * 10) % len(self.map)
        #     if start > end:
        #         start, end = end, start
        if offset is None:
            offset = self.offset

        self.current_index = self.closest_point(lat0, long0, self.map, start, end)
        self.current_index = self.current_index % len(self.map)  # the closest index on the map
        if goal_num is None:
            self.goal_index = (self.current_index + offset) % len(self.map)  # set goal to the next checkpoint
        else:
            self.goal_index = goal_num

        self.goal_lat, self.goal_long = self.map[self.goal_index]

        return self.goal_index

    def closest_point(self, lat0, long0, gps_map, start=0, end=None, init=False):
        smallest_dist = None
        goal_index = 0

        if end is None:
            end = len(gps_map)

        if init:
            index_range = self.init_indices
        else:
            index_range = range(start, end)

        for index in index_range:
            lat1, long1 = gps_map[index]
            dist = math.sqrt((lat1 - lat0) * (lat1 - lat0) + (long1 - long0) * (long1 - long0))
            if smallest_dist is None or dist < smallest_dist:
                smallest_dist = dist
                goal_index = index
        return goal_index

    def lock_onto_map(self, lat, long, init):
        index = self.closest_point(lat, long, self.map, init=init)
        return self.map[index][0], self.map[index][1], index

    @staticmethod
    def does_point_intersect(point: list, edge_1: list, edge_2: list, epsilon=1E-5):
        if edge_1[1] > edge_2[1]:
            edge_1, edge_2 = edge_2, edge_1
        if point[1] == edge_1[1] or point[1] == edge_2[1]:
            point[1] += epsilon

        if (point[1] > edge_2[1] or point[1] < edge_1[1]) or (point[0] > max(edge_1[0], edge_2[0])):
            return False

        if point[0] < min(edge_1[0], edge_2[0]):
            return True
        else:
            if abs(edge_1[0] - edge_2[0]) > MapManipulator.MIN_FLOAT:
                var1 = (edge_2[1] - edge_1[1]) / (edge_2[0] - edge_1[0])
            else:
                var1 = MapManipulator.MAX_FLOAT

            if abs(edge_1[0] - point[0]) > MapManipulator.MIN_FLOAT:
                var2 = (point[1] - edge_1[1]) / (point[0] - edge_1[0])
            else:
                var2 = MapManipulator.MAX_FLOAT

            return var2 >= var1

    @staticmethod
    def point_within_border(coordinate, gps_map):
        sum_value = 0
        for index in range(len(gps_map) - 1):
            sum_value += MapManipulator.does_point_intersect(
                coordinate, gps_map[index], gps_map[index + 1]
            )
        return sum_value % 2 == 1

    def point_inside_outer_map(self, lat, long):
        if self.outer_map is not None:
            return MapManipulator.point_within_border([lat, long], self.outer_map)
        else:
            return True

    def point_outside_inner_map(self, lat, long):
        if self.inner_map is not None:
            return not MapManipulator.point_within_border([lat, long], self.inner_map)
        else:
            return True

    @staticmethod
    def shift_angle(angle):
        """
        Shift the angle error such that 0 degrees is pointed forward.
        This eliminates wrap around errors (the robot won't spin 350
        when told to spin -10, it will spin -10 degrees).
        Basically turns a relative goal angle (goal - current) with a range of
        0...2 * pi to a range of -pi...pi
        """

        while angle >= 2 * math.pi:
            angle -= 2 * math.pi

        while angle < 0:
            angle += 2 * math.pi

        if math.pi < angle < 2 * math.pi:
            return -angle + 2 * math.pi
        else:
            return -angle

    def __len__(self):
        return len(self.map)


class PID:
    def __init__(self, kp, kd, ki, lower_limit=None, upper_limit=None):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        # self.prev_value = None
        self.prev_error = 0
        self.error_sum = 0
        self.prev_time = 0
        self.lower_limit = lower_limit
        self.upper_limit = upper_limit

    def update(self, error, timestamp):
        # if self.prev_value is None:
        #     self.prev_value = value
        #     return 0.0

        dt = timestamp - self.prev_time

        # error = goal - value
        deriv = (error - self.prev_error) / dt
        self.error_sum += error * dt

        # self.prev_value = value
        self.prev_error = error
        self.prev_time = timestamp

        output = self.kp * error + self.kd * deriv + self.ki * self.error_sum
        if self.upper_limit is not None and output > self.upper_limit:
            output = self.upper_limit
        if self.lower_limit is not None and output < self.lower_limit:
            output = self.lower_limit
        return output


if __name__ == '__main__':
    def ray_test():
        triangle_inner_map = [[0, 0], [10, 0], [0, 10]]
        triangle_outer_map = [[-10, -10], [20, 0], [0, 20]]

        print(MapManipulator.point_within_border([0, 0], triangle_inner_map))
        print(MapManipulator.point_within_border([0, 0], triangle_outer_map))


    ray_test()
