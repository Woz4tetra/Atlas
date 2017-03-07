import math
import sys


class BozoController:
    MAX_FLOAT = sys.float_info.max
    MIN_FLOAT = sys.float_info.min

    def __init__(self, course_map, inner_border_map=None, outer_border_map=None, offset=1, border_skip_check=10):
        self.map = course_map
        self.inner_map = inner_border_map
        self.outer_map = outer_border_map

        self.offset = offset
        self.border_skip_check = border_skip_check
        self.current_index = None
        self.current_angle = 0.0

    def initialize(self, lat, long):
        self.current_index = self.get_goal(lat, long, 0, len(self.map))

    def is_initialized(self):
        return self.current_index is not None

    def update(self, lat, long, yaw):
        goal_index = self.get_goal(lat, long, 0, -1)
        goal_lat, goal_long = self.map[goal_index]

        goal_angle = math.atan2(goal_long - long, goal_lat - lat) - math.pi / 2
        if goal_angle < 0:
            goal_angle += 2 * math.pi
        angle_error = self.shift_angle(goal_angle - yaw)

        self.current_angle = angle_error
        return self.current_angle

    def get_goal(self, lat0, long0, start_index, end_index):
        smallest_dist = None
        goal_index = 0

        start = min(start_index % len(self.map), end_index % len(self.map))
        end = min(end_index % len(self.map), end_index % len(self.map))
        # print(start, end)
        for index in range(start, end):  # only search a few indices ahead
            lat1, long1 = self.map[index]
            dist = math.sqrt((lat1 - lat0) * (lat1 - lat0) + (long1 - long0) * (long1 - long0))
            if smallest_dist is None or dist < smallest_dist:
                smallest_dist = dist
                goal_index = index

                # if smallest_dist is not None:
                #     print("%0.4f" % smallest_dist, end=", ")
        # if smallest_dist is not None:
        #     print("%0.4f" % smallest_dist, start, end)


        # print(goal_index, end=", ")
        goal_index = (goal_index + self.offset) % len(self.map)  # set goal to the next checkpoint
        # print(goal_index)
        self.current_index = goal_index % len(self.map)  # the closest index on the map

        # print("%i, %0.6f, %0.6f" % (goal_index, x0, y0))
        # print("%0.6f, %0.6f" % (self.map[goal_index][0], self.map[goal_index][1]))

        return goal_index

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
            if abs(edge_1[0] - edge_2[0]) > BozoController.MIN_FLOAT:
                var1 = (edge_2[1] - edge_1[1]) / (edge_2[0] - edge_1[0])
            else:
                var1 = BozoController.MAX_FLOAT

            if abs(edge_1[0] - point[0]) > BozoController.MIN_FLOAT:
                var2 = (point[1] - edge_1[1]) / (point[0] - edge_1[0])
            else:
                var2 = BozoController.MAX_FLOAT

            return var2 >= var1

    @staticmethod
    def point_within_border(coordinate, gps_map):
        sum_value = 0
        for index in range(len(gps_map) - 1):
            sum_value += BozoController.does_point_intersect(
                coordinate, gps_map[index], gps_map[index + 1]
            )
        return sum_value % 2 == 1

    def is_point_inside(self, lat, long):
        if self.outer_map is not None:
            return BozoController.point_within_border([lat, long], self.outer_map)
        else:
            return True

    def is_point_outside(self, lat, long):
        if self.inner_map is not None:
            return not BozoController.point_within_border([lat, long], self.inner_map)
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


if __name__ == '__main__':
    def ray_test():
        triangle_inner_map = [[0, 0], [10, 0], [0, 10]]
        triangle_outer_map = [[-10, -10], [20, 0], [0, 20]]

        print(BozoController.point_within_border([0, 0], triangle_inner_map))
        print(BozoController.point_within_border([0, 0], triangle_outer_map))

    ray_test()