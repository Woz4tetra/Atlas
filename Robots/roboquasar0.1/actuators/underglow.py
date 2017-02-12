from atlasbuggy.robot.robotobject import RobotObject

class Underglow(RobotObject):
    def __init__(self, enabled=True):
        super(Underglow, self).__init__("underglow", enabled)

    def set_cycle_val(self, value):
        value = abs(int(value))
        # print("cycle value:", value)
        self.send(value)
