import math


class Controller:
    def __init__(self, imaginary_length, front_back_dist, kp, kd):
        self.l = imaginary_length
        self.front_back_dist = front_back_dist
        self.kp = kp
        self.kd = kd

    def update(self, state, goal_x, goal_y):
        x_control = -self.kp * (goal_x - state["x"]) + \
                    -self.kd * state["vx"]
        y_control = -self.kp * (goal_y - state["y"]) + \
                    -self.kd * state["vy"]

        speed_command = math.cos(state["angle"]) * x_control + \
                        math.sin(state["angle"]) * y_control
        ang_v = -math.sin(state["angle"]) / self.l * x_control + \
                math.cos(state["angle"]) / self.l * y_control

        angle_command = math.atan(ang_v * self.front_back_dist / speed_command)

        return speed_command, angle_command
