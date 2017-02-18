import math


class BozoController:
    def __init__(self, course_map, offset=1):
        self.map = course_map
        self.offset = offset
        self.current_index = None
        self.current_angle = 0.0

    def initialize(self, lat, long):
        self.current_index = self.get_goal(lat, long)

    def update(self, lat, long, yaw):
        goal_index = self.get_goal(lat, long)
        goal_lat, goal_long = self.map[goal_index]

        goal_angle = math.atan2(goal_long - long, goal_lat - lat)
        if goal_angle < 0:
            goal_angle += 2 * math.pi
        angle_error = self.shift_angle(goal_angle - yaw)

        self.current_angle = angle_error
        return self.current_angle

    def get_goal(self, lat0, long0):
        smallest_dist = None
        goal_index = 0
        for index in range(self.current_index, self.current_index + 10):  # only search a few indices ahead
            lat1, long1 = self.map[index]
            dist = ((lat1 - lat0) * (lat1 - lat0) +
                    (long1 - long0) * (long1 - long0)) ** 0.5
            if smallest_dist is None or dist < smallest_dist:
                smallest_dist = dist
                goal_index = index

        self.current_index = goal_index % len(self.map)  # the closest index on the map

        goal_index = (goal_index + self.offset) % len(self.map)  # set goal to the next checkpoint
        # print("%i, %0.6f, %0.6f" % (goal_index, x0, y0))
        # print("%0.6f, %0.6f" % (self.map[goal_index][0], self.map[goal_index][1]))

        return goal_index

    @staticmethod
    def shift_angle(angle):
        """
        Shift the angle error such that 0 degrees is pointed forward.
        This eliminates wrap around errors (the robot won't spin 350
        when told to spin -10, it will spin -10 degrees).
        Basically turns a relative goal angle (goal - current) with a range of
        0...2 * pi to a range of -pi...pi
        """

        while angle > 2 * math.pi:
            angle -= 2 * math.pi

        while angle < 0:
            angle += 2 * math.pi

        if math.pi < angle < 2 * math.pi:
            return angle - 2 * math.pi
        else:
            return angle
