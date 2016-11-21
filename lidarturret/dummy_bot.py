import time

import project
from robot.interface import RobotObject, RobotInterface

live = True


class Dummy(RobotObject):
    def __init__(self):
        self.value_1 = 0
        self.value_2 = 0

        self.led_state = False

        self.time0 = time.time()

        super(Dummy, self).__init__("dummy", "/dev/tty.SLAB_USBtoUART")

    def parse_packet(self, packet):
        data = packet.split("\t")
        self.value_1 = int(data[0])
        self.value_2 = int(data[1])

    def command_1(self):
        self.write_packet("t%i" % int(time.time() - self.time0))

    def command_2(self, led_state):
        self.write_packet("l%i" % int(led_state))


if live:
    class DummyBot(RobotInterface):
        def __init__(self):
            self.dummy = Dummy()
            super(DummyBot, self).__init__(self.dummy, log_data=False)

        def main(self):
            if self.dummy.did_update():
                print(self.dummy.value_1, self.dummy.value_2,
                      self.dummy.led_state)

                self.dummy.command_2(self.dummy.led_state)
                self.dummy.led_state = not self.dummy.led_state


    def run_dummy():
        DummyBot().run()


    run_dummy()
else:
    from robot.simulator import Simulator


    class SimulatedDummy(Simulator):
        def __init__(self, file_name, directory, **plot_info):
            self.dummy = Dummy()
            super(SimulatedDummy, self).__init__(
                file_name, directory, plot_info, self.dummy)

        def step(self, index, timestamp, who_i_am, robot_object):
            if who_i_am == self.dummy.who_i_am:
                self.plot_data["plot_dummy"][0].append(robot_object.value_1)
                self.plot_data["plot_dummy"][1].append(robot_object.value_2)


    def simulate_dummy():
        project.set_project_dir("lidarturret")
        SimulatedDummy(-1, -1, plot_dummy=dict(color='red', label="dummy"
                                               )).run()


    simulate_dummy()
