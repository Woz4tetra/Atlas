from atlasbuggy.robot.object import RobotObject


class Blank(RobotObject):
    def __init__(self, enabled=True):
        # define sensor properties

        # replace "blank" with the whoiam ID you want
        # (make sure it matches in the Arduino code, see blank.ino)
        super(Blank, self).__init__("blank", enabled)

    def receive_first(self, packet):
        # parse init packet. Assign to internal properties
        # return None unless something goes wrong
        # overrides RobotObject's receive_first
        pass

    def receive(self, timestamp, packet):
        # parse packets. Assign to internal properties
        # return None unless something goes wrong
        # overrides RobotObject's receive
        pass

    def do_a_thing(self, parameter):
        # example command. If object performs some action, put it in a similar method
        # (please don't call it "do_a_thing" unless that makes sense somehow)
        # DO NOT override the "send" method
        self.send("do a thing:" + str(parameter))
