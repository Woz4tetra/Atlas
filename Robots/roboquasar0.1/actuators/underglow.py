from atlasbuggy.robot.robotobject import RobotObject


class Underglow(RobotObject):
    def __init__(self, enabled=True):
        super(Underglow, self).__init__("underglow", enabled)

    def send_accel(self, accel_x, accel_y):
        # do stuff with accel

        self.send(value)
