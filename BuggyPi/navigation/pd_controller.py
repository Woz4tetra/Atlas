import math


class Controller:
    def __init__(self, imaginary_length, front_back_dist, kp, kd,
                 left_angle_limit, right_angle_limit,
                 left_servo_limit, right_servo_limit):
        self.l = imaginary_length
        self.front_back_dist = front_back_dist
        self.kp = kp
        self.kd = kd

        self.left_angle = left_angle_limit
        self.right_angle = right_angle_limit
        self.left_value = left_servo_limit
        self.right_value = right_servo_limit

    def update(self, state, goal_x, goal_y):
        x_control = -self.kp * (goal_x - state["x"]) + \
                    -self.kd * state["vx"]
        y_control = -self.kp * (goal_y - state["y"]) + \
                    -self.kd * state["vy"]
        
        speed_command = -math.cos(state["angle"]) * x_control - \
                        math.sin(state["angle"]) * y_control
        ang_v = -math.sin(state["angle"]) / self.l * x_control + \
                math.cos(state["angle"]) / self.l * y_control

        angle_command = -math.atan(ang_v * self.front_back_dist / speed_command)
        if angle_command > self.left_value:
            angle_command = self.left_value
        elif angle_command < self.right_value:
            angle_command = self.right_value
        return speed_command, self.angle_to_servo(angle_command)

    def angle_to_servo(self, angle):
        return int(((self.left_value - self.right_value) /
                    (self.left_angle - self.right_angle) *
                    (angle - self.right_angle) + self.right_value))
