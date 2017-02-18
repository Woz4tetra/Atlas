
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
        self.calibrated = False

        super(Steering, self).__init__("steering", enabled)

    def receive_first(self, packet):
        self.max_speed, self.left_limit, self.right_limit = \
            [int(x) for x in packet.split("\t")]

    def receive(self, timestamp, packet):
        if packet == "calibrating":
            print("steering is calibrating")
        elif packet == "done!":
            print("steering calibration complete")
            self.calibrated = True
        else:
            if packet[0] == 'p':
                self.current_step = int(packet[1:])
            elif packet[0] == 'd':
                self.current_step = int(packet[1:])
                self._goal_reached = True

    @property
    def goal_reached(self):
        if self._goal_reached:
            self._goal_reached = False
            return True
        else:
            return False

    def set_speed(self, joystick_value):
        self.speed = int(-joystick_value * self.max_speed)
        self.send("v" + str(self.speed))

    def set_position(self, joystick_value):  # joystick_value: -1.0...1.0
        step = int(joystick_value * self.angle_to_step)
        self.send("p" + str(step))

    def calibrate(self):
        self.send("c")

    def __str__(self):
        string = "%s(whoiam=%s)\n\t" % (self.__class__.__name__, self.whoiam)
        string += "speed: %2.0d, position: %2.0d, moving: %s\n" % (self.speed, self.current_step, not self._goal_reached)
        return string
