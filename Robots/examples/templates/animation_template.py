# Compare checkpoint data to checkpoint maps

from atlasbuggy.interface.simulated import SimulatedRobot

from atlasbuggy.plotters.liveplotter import LivePlotter
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

        # create a live plot with two subplots
        self.plotter = LivePlotter(2, self.some_plot_collection, self.another_plot, matplotlib_events={
            "key_press_event": self.key_press
        })

        super(PlotterTemplate, self).__init__(
            file_name, directory,
            # I recommended putting every important log file name and directory in a dictionary
            self.blank
        )

        self.link(self.blank, self.received_blank_sensor)

    def key_press(self, event):
        # matplotlib callback event
        # pause both the simulated robot and the plot
        if event.key == " ":
            self.toggle_pause()
            self.plotter.toggle_pause()

    def received_blank_sensor(self, timestamp, packet, packet_type):
        pass  # update plots, send commands to other objects, etc...

        # say something went wrong...
        # return "error"

        # if something signalled to exit...
        # return "exit"

        # returning nothing (or None) implies everything is ok

    def received(self, timestamp, whoiam, packet, packet_type):
        # called if any packet is received from the log file

        # update the plot. Put this where appropriate. Minimize it's use as it takes time to plot
        # This is where plot events are updated (such as key presses). So don't leave this out if the plot
        # is paused!
        if self.plotter.plot() is not None:
            return "error"

        if self.did_receive(self.blank):  # alternative to callbacks
            pass

            # say something went wrong...
            # return "error"

            # if something signalled to exit...
            # return "exit"

            # returning nothing (or None) implies everything is ok

    def close(self, reason):
        # if the program isn't exiting because of an error, display the plot
        if reason != "error":
            self.plotter.plot()
            self.plotter.show()


PlotterTemplate().run()
