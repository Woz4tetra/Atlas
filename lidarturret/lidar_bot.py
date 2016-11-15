from robot_interface import RobotObject, RobotInterface


class LidarTurret(RobotObject):
    def __init__(self, ticks_per_rotation=38):
        self.ticks_per_rotation = ticks_per_rotation
        self.current_tick = 0
        self.rotations = 0

        self.point_cloud = [0] * ticks_per_rotation

        super(LidarTurret, self).__init__("lidar", ["/dev/cu.usbmodem*",
                                                    "/dev/tty.usbmodem*"])

    def parse_packet(self, packet):
        data = packet.split("\t")
        if len(data) == 2:
            self.current_tick = int(data[0])
            self.rotations = int(data[1])
        elif len(data) == 1:
            self.point_cloud[self.current_tick] = int(data[0])


class LidarBot(RobotInterface):
    def __init__(self):
        self.lidar = LidarTurret()

        super(LidarBot, self).__init__(self.lidar)

    def main(self):
        if self.lidar.did_update():
            print(self.lidar.point_cloud)
