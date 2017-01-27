"""
Generates motor and servo commands based on current location and a goal position
"""

import math
import time


class PID:
    """
    A standard PID controller with an added lower and upper bound feature.
    For every time step, provide a new error value and the resulting "effort"
    to exert
    """

    def __init__(self, kp, kd, ki, lower_bound, upper_bound):
        self.kp = kp
        self.kd = kd
        self.ki = ki

        self.prev_error = 0
        self.sum_error = 0
        self.prev_time = time.time()

        self.lower_bound = lower_bound
        self.upper_bound = upper_bound

    def update(self, error):
        """
        Provide a new error value based on some metric
        (for example, distance). Depending on what you provide for error,
        a certain "effort" value will be returned. Tune your kp, kd, and ki
        values to get ideal behavior
        """
        dt = time.time() - self.prev_time
        effort = self.kp * error + \
                 self.kd * (error - self.prev_error) / dt + \
                 self.ki * self.sum_error * dt
        self.prev_error = error
        self.sum_error += error
        if abs(effort) < self.lower_bound:
            effort = self.lower_bound * ((effort > 0) - (effort < 0))
        if abs(effort) > self.upper_bound:
            effort = self.upper_bound * ((effort > 0) - (effort < 0))
        self.prev_time = time.time()

        return effort

    def reset(self):
        """Reset the PID controller to initial conditions"""
        self.prev_error = 0
        self.sum_error = 0
        self.prev_time = time.time()


class Controller:
    """
    Takes in a current and goal position and returns the motor and servo values
    required to get there as quickly as possible
    """

    def __init__(self, course_map, offset=1):
        self.map = course_map
        self.offset = offset

    def update(self, lat, long, yaw):
        goal_lat, goal_long = self.get_goal(lat, long)

        goal_angle = math.atan2(goal_long - long, goal_lat - lat)
        if goal_angle < 0:
            goal_angle += 2 * math.pi
        angle_error = self.shift_angle(goal_angle - yaw)
        return angle_error

    def get_goal(self, lat0, long0):
        smallest_dist = None
        goal_index = 0
        for index in range(len(self.map)):
            lat1, long1 = self.map[index]
            dist = ((lat1 - lat0) * (lat1 - lat0) +
                    (long1 - long0) * (long1 - long0)) ** 0.5
            if smallest_dist is None or dist < smallest_dist:
                smallest_dist = dist
                goal_index = index
        goal_index = (goal_index + self.offset) % len(self.map)
        # print("%i, %0.6f, %0.6f" % (goal_index, x0, y0))
        # print("%0.6f, %0.6f" % (self.map[goal_index][0], self.map[goal_index][1]))
        return self.map[goal_index]

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
