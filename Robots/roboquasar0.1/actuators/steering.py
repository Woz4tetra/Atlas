import math
import time
from atlasbuggy.robot.object import RobotObject


class Steering(RobotObject):
    def __init__(self, enabled=True):
        self.current_step = 0
        self.goal_step = 0
        self.speed = 0.0

        # steps / (arc length / radius)
        self.angle_to_step = 136 / (1.5 / 5)
        self.max_speed = 0.0
        self.left_limit = 0
        self.right_limit = 0

        self.left_limit_angle = 0
        self.right_limit_angle = 0

        self._goal_reached = False
        self.moving = False
        self.calibrated = False

        self.sent_step = 0
        self.sent_angle = 0.0

        self.last_update_t = time.time()

        super(Steering, self).__init__("steering", enabled)

    def receive_first(self, packet):
        self.max_speed, self.left_limit, self.right_limit = \
            [int(x) for x in packet.split("\t")]
        self.left_limit_angle = self.left_limit / self.angle_to_step
        self.right_limit_angle = self.right_limit / self.angle_to_step

    def receive(self, timestamp, packet):
        if packet == "calibrating":
            # print("steering is calibrating")
            self.moving = True
        elif packet == "done!":
            # print("steering calibration complete")
            self.calibrated = True
            self.moving = False
            self.last_update_t = time.time()
        else:
            if packet[0] == 'p':
                self.current_step = int(packet[1:])
                self.moving = True
            elif packet[0] == 'd':
                self.current_step = int(packet[1:])
                self.moving = False
            elif packet[0] == 'g':
                self.goal_step = int(packet[1:])

    @property
    def goal_reached(self):
        if self._goal_reached:
            self._goal_reached = False
            return True
        else:
            return False

    def set_speed(self, speed_value):
        speed = int(-speed_value * self.max_speed)
        if abs(speed) > self.max_speed / 2 or speed == 0:
            self.speed = speed
            self.moving = self.speed != 0
            print("speed:", self.speed)
            self.send("v" + str(self.speed))

    def change_step(self, delta_step):
        self.send_step(self.current_step + delta_step)

    def change_position(self, delta_angle):
        self.send_step(self.current_step + delta_angle * self.angle_to_step)

    def set_position(self, goal_angle):
        # if time.time() - self.last_update_t < 0.5:
        #     return
        # self.last_update_t = time.time()
        # if not self.moving:
        self.sent_angle = goal_angle
        self.send_step(goal_angle * self.angle_to_step)

    def send_step(self, step):
        self.sent_step = int(step)
        if self.sent_step < self.left_limit:
            self.sent_step = self.left_limit
        elif self.sent_step > self.right_limit:
            self.sent_step = self.right_limit
        self.send("p" + str(self.sent_step))

    def calibrate(self):
        self.send("c")

    def __str__(self):
        string = "%s(whoiam=%s)\n\t" % (self.__class__.__name__, self.whoiam)
        string += "speed: %2.0d, position: %2.0d, goal: %2.0d, moving: %s\n" % (
        self.speed, self.current_step, self.goal_step, self.moving)
        return string
