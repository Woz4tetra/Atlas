import time


class PID:
    def __init__(self, kp, kd, ki):
        """

        :param current_x: Assumes constant since forward velocity isn't used
          for this PID, only angle
        :param kp:
        :param kd:
        :param ki:

        """
        self.prev_error = 0.0
        self.sum_error = 0.0
        self.time0 = time.time()

        self.kp, self.kd, self.ki = kp, kd, ki

    def update(self, angle_error):
        """
        Returns goal angle of the servo in radians
        """
        dt = time.time() - self.time0

        self.sum_error += angle_error

        derivative = (angle_error - self.prev_error) / dt
        integral = self.sum_error / dt

        input_state = self.kp * angle_error + self.kd * derivative + self.ki * integral

        self.prev_error = angle_error
        self.time0 = time.time()

        return input_state
