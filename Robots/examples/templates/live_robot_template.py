# template for running a live robot
import argparse

from atlasbuggy.interface.live import LiveRobot

from my_joystick import ExampleJoystick

from atlasbuggy.plotters.liveplotter import LivePlotter
from atlasbuggy.plotters.plot import RobotPlot

# import robot objects
from robot_objects.blank.blank import Blank

parser = argparse.ArgumentParser()
parser.add_argument("-l", "--nolog", help="disable logging", action="store_false")
args = parser.parse_args()


class MyRobot(LiveRobot):
    def __init__(self):
        # initialize robot objects
        self.blank = Blank()

        self.enable_plotting = False
        if self.enable_plotting:
            self.plot1 = RobotPlot("plot name")
            self.plotter = LivePlotter(
                1,
                self.plot1
                # put robot plots here
            )

        super(MyRobot, self).__init__(
            # pass initialized robot objects here
            joystick=ExampleJoystick(),
            log_data=args.nolog,  # if no command line arguments are given, data will be logged
            log_dir=("subfolder", None),  # None means use today's date
            debug_prints=True
        )

        self.link(
            self.blank,  # robot object
            self.received_blank_sensor  # callback function
        )

    def received_blank_sensor(self, timestamp, packet, packet_type):
        pass  # update plots, send commands to other objects, etc...

        # say something went wrong...
        # return "error"

        # if something signalled to exit...
        # return "exit"

        # returning nothing (or None) implies everything is ok

    def received(self, timestamp, whoiam, packet, packet_type):
        # called if any packet is received

        if self.did_receive(self.blank):  # alternative to callbacks
            pass

            # say something went wrong...
            # return "error"

            # if something signalled to exit...
            # return "exit"

        # returning nothing (or None) implies everything is ok

    def loop(self):
        if self.joystick is not None:
            if self.joystick.axis_updated("left x"):
                self.blank.do_a_thing(self.joystick.get_axis("left x"))

            if self.joystick.button_updated("X") and self.joystick.get_button("X"):
                self.blank.do_a_thing(self.joystick.get_button("X"))

    def close(self, reason):
        if reason != "done":
            # something went wrong. Try to stop the robot safely
            print("!!EMERGENCY BRAKE!!")


MyRobot().run()
