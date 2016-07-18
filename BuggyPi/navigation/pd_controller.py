# class Controller:
#     def __init__(self, imaginary_length, front_back_dist, kp, kd,
#                  left_angle_limit, right_angle_limit,
#                  left_servo_limit, right_servo_limit):
#         self.l = imaginary_length
#         self.front_back_dist = front_back_dist
#         self.kp = kp
#         self.kd = kd
#
#         self.left_angle = left_angle_limit
#         self.right_angle = right_angle_limit
#         self.left_value = left_servo_limit
#         self.right_value = right_servo_limit
#
#     def update(self, state, goal_x, goal_y):
#         x_control = -self.kp * (goal_x - state["x"]) + \
#                     -self.kd * state["vx"]
#         y_control = -self.kp * (goal_y - state["y"]) + \
#                     -self.kd * state["vy"]
#
#         speed_command = -math.cos(state["angle"]) * x_control - \
#                         math.sin(state["angle"]) * y_control
#         ang_v = -math.sin(state["angle"]) / self.l * x_control + \
#                 math.cos(state["angle"]) / self.l * y_control
#
#         angle_command = -math.atan(ang_v * self.front_back_dist / speed_command)
#         if angle_command > self.left_value:
#             angle_command = self.left_value
#         elif angle_command < self.right_value:
#             angle_command = self.right_value
#         return speed_command, self.angle_to_servo(angle_command)
#
#     def angle_to_servo(self, angle):
#         return int(((self.left_value - self.right_value) /
#                     (self.left_angle - self.right_angle) *
#                     (angle - self.right_angle) + self.right_value))

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
                self.kd * (error - self.prev_error) + \
                self.ki * self.sum_error * dt
        self.prev_error = error
        self.sum_error += error
        if abs(power) < self.lower_bound:
            power = self.lower_bound * (power > 0) - (power < 0)
        if abs(power) > self.upper_bound:
            power = self.upper_bound * (power > 0) - (power < 0)

        self.prev_time = time.time()

        return power


class Controller:
    def __init__(self, angle_kp, angle_kd, angle_ki,
                 left_angle_limit, right_angle_limit,
                 dist_kp, dist_kd, dist_ki,
                 lower_speed_limit, upper_speed_limit):
        self.angle_pid = PID(angle_kp, angle_kd, angle_ki,
                             left_angle_limit, right_angle_limit)
        self.dist_pid = PID(dist_kp, dist_kd, dist_ki,
                            lower_speed_limit, upper_speed_limit)

    def update(self, state, goal_x, goal_y):
        goal_angle = math.atan2(goal_y, goal_x)
        angle_error = self.shift_angle(goal_angle - state["angle"])

        angle_command = self.angle_pid.update(angle_error)

        error_dist = ((goal_x - state["x"]) ** 2 +
                      (goal_y - state["y"]) ** 2) ** 0.5
        speed_command = self.dist_pid.update(error_dist)

        print(("%0.4f" * 3) % (goal_angle, angle_error, error_dist))
        return angle_command, speed_command

    @staticmethod
    def shift_angle(angle):
        # Shift the angle error such that 0 degrees is pointed forward.
        # This eliminates wrap around errors (the robot won't spin 350
        # when told to spin -10, it will spin -10 degrees)
        while angle > 2 * math.pi:
            angle -= 2 * math.pi

        while angle < 0:
            angle += 2 * math.pi

        if math.pi < angle < 2 * math.pi:
            return angle - 2 * math.pi
        else:
            return angle
