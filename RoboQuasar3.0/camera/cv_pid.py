import time
import numpy as np

class PID:
    def __init__(self, current_x, kp, kd, ki):
        """

        :param current_x: Assumes constant since forward velocity isn't used
          for this PID, only angle
        :param current_y: Same as x
        :param kp:
        :param kd:
        :param ki:

        """
        self.curr_state = current_x
        self.prev_error = 0.0
        self.sum_error = 0.0
        self.time0 = time.time()

        self.kp, self.kd, self.ki = kp, kd, ki

    def update(self, goal_x):
        """
        Returns goal angle of the servo in radians

        """
        dt = time.time() - self.time0

        error = goal_x - self.curr_state
        self.sum_error += error

        derivative = (error - self.prev_error) / dt
        integral = self.sum_error / dt

        input_state = self.kp * error + self.kd * derivative + self.ki * integral

        self.prev_error = error
        self.time0 = time.time()

        return input_state
