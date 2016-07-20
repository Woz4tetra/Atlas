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
    def __init__(self,
##                 angle_kp, angle_kd, angle_ki,
##                 left_angle_limit, right_angle_limit,
                 dist_kp, dist_kd, dist_ki,
                 lower_speed_limit, upper_speed_limit):
##        if left_angle_limit < right_angle_limit:
##            lower_servo_bound = left_angle_limit
##            upper_servo_bound = right_angle_limit
##        else:
##            lower_servo_bound = right_angle_limit
##            upper_servo_bound = left_angle_limit
##
##        self.angle_pid = PID(angle_kp, angle_kd, angle_ki,
##                             lower_servo_bound, upper_servo_bound)
        self.dist_pid = PID(dist_kp, dist_kd, dist_ki,
                            lower_speed_limit, upper_speed_limit)

    def update(self, state, goal_x, goal_y):
        goal_angle = math.atan2(goal_y - state["y"], goal_x - state["x"])
        if goal_angle < 0:
            goal_angle += 2 * math.pi
        angle_error = self.shift_angle(goal_angle - state["angle"])
##        angle_command = self.angle_pid.update(angle_error)

        error_dist = ((goal_x - state["x"]) ** 2 +
                      (goal_y - state["y"]) ** 2) ** 0.5
        speed_command = self.dist_pid.update(error_dist)

##        print(("%0.4f\t" * 5) % (goal_x, state["x"], goal_y, state["y"], state["angle"]))
##        print(("%0.4f\t" * 3) % (goal_angle, angle_error, error_dist))
        return angle_error, speed_command

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

    def reset(self):
##        self.angle_pid.reset()
        self.dist_pid.reset()


if __name__ == '__main__':
    def test():
        left_angle_limit = 0.81096
        right_angle_limit = -0.53719
        left_servo_limit = 35
        right_servo_limit = -25

        def angle_to_servo(angle):
            return int(((left_servo_limit - right_servo_limit) /
                        (left_angle_limit - right_angle_limit) *
                        (angle - right_angle_limit) + right_servo_limit))

        test_controller = Controller(1, 1, 1, right_angle_limit, left_angle_limit,
                                     100000, 10000, 10000, 0.0, 1.0)
        print("left:", left_servo_limit, "right:", right_servo_limit)

        for x, y in [(0, 0), (1e-06, -1e-07), (1e-06, -1e-07),
                     (1e-06, -1e-07), (1e-06, -1e-07),(2e-06, -2e-07),
                     (3e-06, -3e-07), (4e-06, -4e-07), (5e-06, -5e-07),
                     (6e-06, -6e-07), (7e-06, -7e-07), (8e-06, -8e-07),
                     (9e-06, -9e-07), (1e-05, -1e-06), (1e-05, -1e-06)]:
            angle_command, speed_command = test_controller.update({
                "x": x, "y": y, "angle": 0.15,
            }, 1e-05, -1e-06)
            time.sleep(0.1)

            speed_command = int(speed_command * 100)
            if 8 < speed_command < 40:
                speed_command = 40
            elif speed_command <= 8:
                speed_command = 0
            print(angle_to_servo(angle_command), speed_command)


    test()
