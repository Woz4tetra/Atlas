# Compare checkpoint data to checkpoint maps

from atlasbuggy.interface.simulated import SimulatedRobot

from atlasbuggy.plotters.staticplotter import StaticPlotter
from atlasbuggy.plotters.plot import RobotPlot
from atlasbuggy.plotters.collection import RobotPlotCollection

from robot_objects.blank.blank import Blank


class PlotterTemplate(SimulatedRobot):
    def __init__(self):
        self.blank = Blank()

        self.some_plot = RobotPlot("some plot")
        self.other_plot = RobotPlot("some other plot")

        self.another_plot = RobotPlot("another plot")

        # put plots on the same subplot
        self.some_plot_collection = RobotPlotCollection(
            "plot collection", self.some_plot, self.other_plot,
        )

        # create a figure with two subplots
        self.plotter = StaticPlotter(2, self.some_plot_collection, self.another_plot)

        super(PlotterTemplate, self).__init__(
            file_name, directory,  # I recommended putting every important log file name and directory in a dictionary
            self.blank
        )

        self.link(self.blank, self.received_blank_sensor)

    def received_blank_sensor(self, timestamp, packet, packet_type):
        pass  # update plots, send commands to other objects, etc...

        # say something went wrong...
        # return "error"

        # if something signalled to exit...
        # return "exit"

        # returning nothing (or None) implies everything is ok

    def received(self, timestamp, whoiam, packet, packet_type):
        # called if any packet is received from the log file

        if self.did_receive(self.blank):  # alternative to callbacks
            pass

            # say something went wrong...
            # return "error"

            # if something signalled to exit...
            # return "exit"

            # returning nothing (or None) implies everything is ok

    def loop(self):
        # update the plot if the simulated robot is paused
        if self.is_paused:
            if self.plotter.plot() is not None:
                return "error"

    def close(self, reason):
        # if the program isn't exiting because of an error, display the plot
        if reason != "error":
            self.plotter.plot()
            self.plotter.show()


PlotterTemplate().run()
