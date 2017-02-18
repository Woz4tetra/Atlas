from atlasbuggy.robot.object import RobotObject


class Steering(RobotObject):
    def __init__(self, enabled=True):
        self.current_step = 0

        self.speed = 0.0
        self.angle_to_step = 1.0  # find this value
        self.max_speed = 0.0
        self.left_limit = 0
        self.right_limit = 0

        self._goal_reached = False
        self.moving = False
        self.calibrated = False

        super(Steering, self).__init__("steering", enabled)

    def receive_first(self, packet):
        self.max_speed, self.left_limit, self.right_limit = \
            [int(x) for x in packet.split("\t")]

    def receive(self, timestamp, packet):
        if packet == "calibrating":
            print("steering is calibrating")
            self.moving = True
        elif packet == "done!":
            print("steering calibration complete")
            self.calibrated = True
            self.moving = False
        else:
            if packet[0] == 'p':
                self.current_step = int(packet[1:])
                self.moving = True
            elif packet[0] == 'd':
                self.current_step = int(packet[1:])
                self.moving = False

    @property
    def goal_reached(self):
        if self._goal_reached:
            self._goal_reached = False
            return True
        else:
            return False

    def set_speed(self, speed_value):
        self.speed = int(-speed_value * self.max_speed)
        self.moving = self.speed != 0
        print("speed:", self.speed)
        self.send("v" + str(self.speed))

    def set_position(self, goal_angle):
        step = int(goal_angle * self.angle_to_step)
        self.send("p" + str(step))

    def calibrate(self):
        self.send("c")

    def __str__(self):
        string = "%s(whoiam=%s)\n\t" % (self.__class__.__name__, self.whoiam)
        string += "speed: %2.0d, position: %2.0d, moving: %s\n" % (self.speed, self.current_step, self.moving)
        return string
