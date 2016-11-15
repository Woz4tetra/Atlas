import time
from robot_interface import RobotObject, RobotInterface


class LidarTurret(RobotObject):
    def __init__(self, ticks_per_rotation=38):
        self.ticks_per_rotation = ticks_per_rotation
        self.current_tick = 0
        self.rotations = 0

        self.point_cloud = [0] * ticks_per_rotation

        super(LidarTurret, self).__init__(0, ["/dev/cu.usbmodem*",
                                              "/dev/tty.usbmodem*"])

    def parse_packet(self, packet):
        data = packet.split("\t")
        if len(data) == 2:
            self.current_tick = int(data[0])
            self.rotations = int(data[1])
        elif len(data) == 1:
            self.point_cloud[self.current_tick] = int(data[0])


class Dummy(RobotObject):
    def __init__(self):
        self.value_1 = 0
        self.value_2 = 0

        super(Dummy, self).__init__(1, "/dev/tty.SLAB_USBtoUART")

    def parse_packet(self, packet):
        data = packet.split("\t")
        self.value_1 = int(data[0])
        self.value_2 = int(data[1])


class LidarBot(RobotInterface):
    def __init__(self):
        # lidar = LidarTurret()
        self.dummy = Dummy()

        super(LidarBot, self).__init__(self.dummy)

        self.time0 = time.time()

    def main(self):
        if self.dummy.did_update():
            print(int((time.time() - self.time0) * 1000), self.dummy.value_1,
                  self.dummy.value_2)


LidarBot().run()
