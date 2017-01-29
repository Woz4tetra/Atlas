
from atlasbuggy.robot.robotobject import RobotObject


class Steering(RobotObject):
    def __init__(self, enabled=True):
        self.step_num = None
        self.speed = 0.0
        self.angle_to_step = 1.0  # find this value

        super(Steering, self).__init__("steering", enabled)

    def receive_first(self, packet):
        self.step_num = int(packet)

    def receive(self, timestamp, packet):
        self.step_num = int(packet)

    def set_speed(self, speed):
        self.speed = int(speed)
        self.send("s" + str(self.speed))

    def set_position(self, joystick_value):  # joystick_value: -1.0...1.0
        step = int(joystick_value * self.angle_to_step)
        self.send("p" + str(step))
