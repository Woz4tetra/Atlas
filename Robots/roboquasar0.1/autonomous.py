# This file is meant to collect data from manual control. Terminal interface only
import argparse
import cmd
import math
import time
from threading import Thread

from atlasbuggy.interface.live import RobotRunner

from joysticks.wiiu_joystick import WiiUJoystick
from roboquasar import RoboQuasar, map_sets

parser = argparse.ArgumentParser()
parser.add_argument("-l", "--nolog", help="disable logging", action="store_false")
parser.add_argument("-d", "--debug", help="enable debug prints", action="store_true")
parser.add_argument("-c", "--compass", default=0, help="initialize compass", type=int)
args = parser.parse_args()


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
        runner.exit()
        print()
        return True

    def do_EOF(self, line):
        """
        usage: EOF
        quit the program
        """
        return self.do_q(line)

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

    def do_lights(self, line):
        if len(line) > 0:
            robot.underglow.send('f%s' % line)


log_dir = ("data_days", None)
checkpoint_map_name, inner_map_name, outer_map_name, map_dir = map_sets["cut 3"]

robot = RoboQuasar(False, checkpoint_map_name, inner_map_name, outer_map_name, map_dir, args.compass)
runner = RobotRunner(robot, WiiUJoystick(), log_data=args.nolog, log_dir=log_dir, debug_prints=args.debug)

command_line = AutonomousCommandline()


def run_commands():
    command_line.cmdloop()


t = Thread(target=run_commands)
t.daemon = True
t.start()


runner.run()