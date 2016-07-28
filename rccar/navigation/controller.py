import math
import time


class PID:
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
        dt = time.time() - self.prev_time
        power = self.kp * error + \
                self.kd * (error - self.prev_error) / dt + \
                self.ki * self.sum_error * dt
        self.prev_error = error
        self.sum_error += error
        if abs(power) < self.lower_bound:
            power = self.lower_bound * ((power > 0) - (power < 0))
        if abs(power) > self.upper_bound:
            power = self.upper_bound * ((power > 0) - (power < 0))
        self.prev_time = time.time()

        return power

    def reset(self):
        self.prev_error = 0
        self.sum_error = 0
        self.prev_time = time.time()


class Controller:
    def __init__(self):
        pass

    def update(self, state, goal_x, goal_y):
        goal_angle = math.atan2(goal_y - state["y"], goal_x - state["x"])
        if goal_angle < 0:
            goal_angle += 2 * math.pi
        angle_error = self.shift_angle(goal_angle - state["angle"])
        return angle_error

    @staticmethod
    def shift_angle(angle):
        # Shift the angle error such that 0 degrees is pointed forward.
        # This eliminates wrap around errors (the robot won't spin 350
        # when told to spin -10, it will spin -10 degrees).
        # Basically turns a relative goal angle (goal - current) with a range of
        # 0...2 * pi to a range of -pi...pi

        while angle > 2 * math.pi:
            angle -= 2 * math.pi

        while angle < 0:
            angle += 2 * math.pi

        if math.pi < angle < 2 * math.pi:
            return angle - 2 * math.pi
        else:
            return angle
