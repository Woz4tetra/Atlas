from microcontroller.logger import *
from navigation.buggypi_filter import BuggyPiFilter


class Robot:
    def __init__(self, initial_long=None, initial_lat=None, initial_heading=0.0,
                 counts_per_rotation=6, wheel_radius=0.097,
                 front_back_dist=0.234, max_speed=0.88,
                 left_angle_limit=0.81096, right_angle_limit=-0.53719,
                 left_servo_limit=35, right_servo_limit=-25):
        self.checkpoints = get_map("checkpoints")
        check_long, check_lat = self.checkpoints[0]
        if initial_long is None:
            self.initial_long = check_long
        else:
            self.initial_long = initial_long

        if initial_lat is None:
            self.initial_lat = check_lat
        else:
            self.initial_lat = initial_lat

        self.counts_per_rotation = counts_per_rotation
        self.wheel_radius = wheel_radius
        self.front_back_dist = front_back_dist
        self.max_speed = max_speed
        self.left_angle_limit = left_angle_limit
        self.right_angle_limit = right_angle_limit
        self.left_servo_limit = left_servo_limit
        self.right_servo_limit = right_servo_limit

        self.pi_filter = BuggyPiFilter(
            self.initial_long, self.initial_lat, initial_heading,
            counts_per_rotation, wheel_radius,
            front_back_dist, max_speed,
            left_angle_limit, right_angle_limit,
            left_servo_limit, right_servo_limit
        )

        self.state = self.pi_filter.state
