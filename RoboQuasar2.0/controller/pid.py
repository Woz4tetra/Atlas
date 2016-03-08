# contains the sensor pid controller

import time
import math


class PID:
    def __init__(self):
        self.prev_time = time.time()

        self.prev_error = 0.0
        self.integral = 0.0
        self.derivative = 0.0

        # output will be relative to current position
        self.output = 0.0

        # calibration constants(not calibrated so far)
        self.Kp = 0.11
        self.Ki = 0.0
        self.Kd = 0.0

        self.dC = -0.5

    def update(self, measured_value, set_point):
        """
        outputs the angle given to a servo when given
        current pos in (x,y,theta) and set_point in (x,y)
        """

        time1 = time.time()

        error = self.error_finder(measured_value, set_point)

        dt = time1 - self.prev_time

        self.integral += error * dt
        self.derivative = (error - self.prev_error) / dt

        self.output = (self.Kp * error +
                       self.Ki * self.integral +
                       self.Kd * self.derivative)

        self.prev_error = error
        self.prev_time = time1

        return self.output + measured_value[2]

    def error_finder(self, measured, set_v):
        """
        Takes the tangential distance from next point with current angle,
        divides by dist so has more effect when closer, and the difference
        in angle to get error.

        measured is of the form (x1,y1,theta)
        set_v is of the form (x2,y2)
        """

        x1, y1, theta1 = measured
        x2, y2 = set_v

        dist = ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** (0.5)

        x = (y2 - y1)

        alpha = math.asin(x / dist)
        beta = theta1 - alpha
        tangent = math.sin(beta) * dist

        return self.dC * tangent / dist  # + tC1*(theta2 - theta1)
