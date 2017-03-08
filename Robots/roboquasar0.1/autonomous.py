# This file is meant to collect data from manual control. Terminal interface only
import argparse
import cmd
import math
import time
from threading import Thread

from atlasbuggy.interface import run, close

from joysticks.wiiu_joystick import WiiUJoystick
from roboquasar import RoboQuasar, map_sets

parser = argparse.ArgumentParser()
parser.add_argument("-l", "--nolog", help="disable logging", action="store_false")
parser.add_argument("-d", "--debug", help="enable debug prints", action="store_true")
parser.add_argument("-c", "--compass", default=0, help="initialize compass", type=int)
args = parser.parse_args()

robot = RoboQuasar(False, *map_sets["cut"])
robot.init_compass(args.compass)

class AutonomousCommandline(cmd.Cmd):
    def do_imu(self, line):
        """
        usage: imu
        print the status of the IMU
        """
        print(robot.imu)

    def do_gps(self, line):
        """
        usage: gps
        print the status of the GPS
        """
        print(robot.gps)

    def do_steering(self, line):
        """
        usage: steering
        print the status of the steering
        """
        print(robot.steering)

    def do_brakes(self, line):
        """
        usage: brakes
        print the status of the brakes
        """
        print(robot.brakes)

    def do_b(self, line):
        """
        usage: b
        engage the brake
        """
        robot.brakes.brake()

    def do_r(self, line):
        """
        usage: r
        disengage the brake
        """
        robot.brakes.unbrake()

    def do_s(self, line):
        """
        usage: s [number]
        set the steering to an angle
        """
        if robot.manual_mode:
            try:
                line = math.radians(float(line))
            except ValueError:
                return
            robot.steering.set_position(line)

    def do_manual(self, line):
        """
        usage: manual
        enable manual control
        """
        robot.manual_mode = True

    def do_auto(self, line):
        """
        usage: auto
        enable autonomous control
        """
        robot.manual_mode = False

    def do_q(self, line):
        """
        usage: q
        quit the program
        """
        return True

    def do_compass(self, line):
        """
        usage: compass [number]
        initializes the IMU's offset. Calibrated for iPhone compass input
        """
        try:
            float(line)
        except ValueError:
            return
        robot.init_compass(line)

    def do_wipe(self, line):
        robot.underglow.send('f')

def run_robot():
    run(robot, WiiUJoystick(), log_data=args.nolog, log_dir=("data_days", None), debug_prints=args.debug)

t = Thread(target=run_robot)
t.daemon = True
t.start()

AutonomousCommandline().cmdloop()

close()
