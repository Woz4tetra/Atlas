from gps import GPS
from imu import IMU
from atlasbuggy.robot.interface import RobotInterface
from atlasbuggy.plotters.liveplotter import LivePlotter
from atlasbuggy.plotters.robotplot import RobotPlot


class Runner(RobotInterface):
    def __init__(self):
        self.gps = GPS(enabled=False)
        self.imu = IMU()

        self.imu_plot = RobotPlot("imu data", flat_plot=False)

        self.plotter = LivePlotter(1, self.imu_plot)

        super(Runner, self).__init__(
            self.imu,
            self.gps,
            debug_prints=True,
            log_data=False
        )

    def packet_received(self, timestamp, whoiam, packet):
        if self.did_receive(self.imu):
            # print(timestamp, self.imu.x, self.imu.y, self.imu.z)

            if self.queue_len() < 25:
                self.imu_plot.append(self.imu.x, self.imu.y, self.imu.z)
                if self.plotter.plot() is False:
                    return False
            elif self.did_receive(self.gps):
                print("--")

    def loop(self):
        pass

    def start(self):
        self.plotter.start_time(self.start_time)

    def close(self):
        self.plotter.close()


Runner().run()
