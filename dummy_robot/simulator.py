from atlasbuggy import project
from atlasbuggy.plotters.staticplotter import StaticPlotter
from atlasbuggy.robot.simulator import RobotInterfaceSimulator

from dummy.dummy_bot import Dummy


class DummySimulator(RobotInterfaceSimulator):
    def __init__(self, file_name, directory, start_index=0, end_index=-1):
        self.dummy = Dummy()

        super(DummySimulator, self).__init__(
            file_name, directory, start_index, end_index,
            self.dummy
        )

        # self.dummy.xyz_plot.skip_count = 100
        # self.dummy.xtz_plot.skip_count = 100

        self.dummy_plotter = StaticPlotter(2, self.dummy.xyz_plot, self.dummy.xtz_plot)
        self.time_plotter = StaticPlotter(1, self.dummy.time_plot)

    def object_packet(self, timestamp):
        if self.did_receive(self.dummy.whoiam):
            self.dummy.xyz_plot.append(self.dummy.accel_x, self.dummy.accel_y, self.dummy.accel_z)
            self.dummy.xtz_plot.append(self.dummy.accel_x, self.dummy.dt, self.dummy.accel_z)

        self.print_percent()

    def user_packet(self, timestamp, packet):
        if self.did_receive("time lag"):
            data = packet.split("\t")
            packet_timestamp = float(data[0])
            queue_len = float(data[1])

            self.dummy.time_lag.append(self.dt, packet_timestamp - self.dummy.dt)
            self.dummy.queue_count.append(self.dt, queue_len)

        self.print_percent()

    def command_packet(self, timestamp, packet):
        if self.did_receive(self.dummy.whoiam):
            self.dummy.parse_command(packet)
            print("r: %i, g: %i, y: %i, b: %i" % (
                self.dummy.leds[0], self.dummy.leds[1], self.dummy.leds[2], self.dummy.blue_led))


    def close(self):
        print("plotting...")
        self.dummy_plotter.plot()
        self.time_plotter.plot()
        self.time_plotter.show()


def simulate_dummy():
    file_name, directory = project.parse_arguments(-1, -1)
    DummySimulator(file_name, directory).run()


simulate_dummy()
