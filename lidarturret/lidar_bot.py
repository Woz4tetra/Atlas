from robot_interface import RobotObject, RobotInterface
from robot_simulator import Simulator
import project


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


class Dummy(RobotObject):
    def __init__(self):
        self.value_1 = 0
        self.value_2 = 0

        super(Dummy, self).__init__("dummy", "/dev/tty.SLAB_USBtoUART")

    def parse_packet(self, packet):
        data = packet.split("\t")
        self.value_1 = int(data[0])
        self.value_2 = int(data[1])


class LidarBot(RobotInterface):
    def __init__(self):
        self.lidar = LidarTurret()

        super(LidarBot, self).__init__(self.lidar)

    def main(self):
        if self.lidar.did_update():
            print(self.lidar.point_cloud)


class DummyBot(RobotInterface):
    def __init__(self):
        self.dummy = Dummy()

        super(DummyBot, self).__init__(self.dummy, log_data=False)

    def main(self):
        if self.dummy.did_update():
            print(self.dummy.value_1, self.dummy.value_2)


class SimulatedDummy(Simulator):
    def __init__(self, file_name, directory, **plot_info):
        self.dummy = Dummy()

        super(SimulatedDummy, self).__init__(
            file_name, directory, plot_info, self.dummy)

    def step(self, index, timestamp, who_i_am, robot_object):
        if who_i_am == self.dummy.who_i_am:
            self.plot_data["plot_dummy"][0].append(robot_object.value_1)
            self.plot_data["plot_dummy"][1].append(robot_object.value_2)


def run_dummy():
    DummyBot().run()


def simulate_dummy():
    project.set_project_dir("lidarturret")
    SimulatedDummy(-1, -1, plot_dummy=dict(color='red', label="dummy"
                                           )).run()

run_dummy()
# simulate_dummy()
