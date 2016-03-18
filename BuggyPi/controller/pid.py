# contains the sensor pid controller

import time
import math

class PID:
    def __init__(self):
        self.prev_time = time.time() - 0.001

        self.prev_error = 0.0
        self.integral = 0.0
        self.derivative = 0.0

        # output will be relative to current position
        self.output = 0.0

        # calibration constants(not calibrated so far)
        self.prop_const = 0.11
        self.int_const = 0.0000
        self.deriv_const = 0.0000

    def update(self, measured_value, set_point):
        """
        outputs the angle given to a servo when given
        current pos in (x,y,theta) and set_point in (x,y)
        """

        time1 = time.time()

        # print measured_value, "measured values"
        print(measured_value[0], "\t", measured_value[1], "\t", measured_value[
            2])

        error = PID.error_finder(measured_value, set_point)
        # print error, "error"

        dt = time1 - self.prev_time
        # print dt, ": dt"

        self.integral = self.integral + error * dt
        # print self.integral * self.int_const, "calibrated int"

        self.derivative = (error - self.prev_error) / dt
        # print self.derivative * self.deriv_const, "calibrated deriv"
        # print self.prop_const * error, "calibrated proportion"

        self.output = (self.prop_const * error +
                       self.int_const * self.integral +
                       self.deriv_const * self.derivative)

        self.prev_error = error
        self.prev_time = time1

        return self.output + measured_value[2]

    @staticmethod
    def error_finder(measured, set_v):
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

        dC = -5.0
        # tC1 = -3.0
        # tC2 = -3.0/(dist**0.25)

        # print dist, "dist"
        # print dC*tangent, ": error dist"
        # print ""
        # print tC*(theta2 - theta1), ": error angle"
        return dC * tangent / dist  # + tC1*(theta2 - theta1)

