from tutorial2 import GPS
from atlasbuggy.robot.interface import RobotInterface

from atlasbuggy.plotters.liveplotter import LivePlotter
from atlasbuggy.plotters.robotplot import RobotPlot


class TutRunner(RobotInterface):
    def __init__(self):
        self.tut = GPS()
        self.time_plot = RobotPlot("gps time")
        self.minute_plot = RobotPlot("gps minutes")
        self.plotter = LivePlotter(2, self.time_plot, self.minute_plot)

        super(TutRunner, self).__init__(
            self.tut,
            debug_prints=True,
            log_data=False,
        )

    def loop(self):
        pass

    def packet_received(self, timestamp, whoiam, packet):
        if self.did_receive(self.tut):
            if self.queue_len() < 25:
                self.time_plot.append(timestamp, self.tut.second)
                self.minute_plot.append(timestamp, self.tut.minute)
                if not self.plotter.plot():
                    return False

    def close(self):
        self.plotter.close()

    def start(self):
        self.plotter.start_time(self.start_time)


TutRunner().run()
