# This file is meant to collect data from manual control. Terminal interface only
import argparse
import cmd
import math
import time
from threading import Thread

from atlasbuggy.interface.live import RobotRunner

from joysticks.wiiu_joystick import WiiUJoystick
from roboquasar import RoboQuasar, map_sets
from camera_guidance_test import CameraGuidanceTest

parser = argparse.ArgumentParser()
parser.add_argument("-l", "--nolog", help="disable logging", action="store_false")
parser.add_argument("-d", "--debug", help="enable debug prints", action="store_true")
parser.add_argument("-c", "--compass", default=0, help="initialize compass", type=int)
parser.add_argument("-cam", "--nocamera", help="disable cameras", action="store_false")
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
        robot.brakes.pull()

    def do_r(self, line):
        """
        usage: r
        disengage the brake
        """
        robot.brakes.release()

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

    def do_control(self, line):
        if robot.gps_imu_control_enabled:
            print("gps & imu")
        else:
            if robot.left_pipeline.safety_value > robot.left_pipeline.safety_threshold:
                print("left", end=" ")
            if robot.right_pipeline.safety_value > robot.right_pipeline.safety_threshold:
                print("right", end=" ")
            print("camera")

    def do_q(self, line):
        """
        usage: q
        quit the program
        """
        runner.exit()
        print()
        return True

    def do_showcam(self, line):
        robot.left_camera.show = True
        robot.right_camera.show = True

    def do_hidecam(self, line):
        robot.left_camera.show = False
        robot.right_camera.show = False

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
        robot.bozo_filter.init_compass(line)

    def do_lights(self, line):
        if len(line) > 0:
            robot.underglow.send('f%s' % line)

    def do_pipeangle(self, line):
        print(robot.pipeline_angle)


log_dir = ("push_practice", None)
checkpoint_map_name, inner_map_name, outer_map_name, map_dir = map_sets["buggy"]

robot = RoboQuasar(False, checkpoint_map_name, inner_map_name, outer_map_name, map_dir, args.compass,
                   enable_cameras=args.nocamera, day_mode=False)
# robot = CameraGuidanceTest(enable_cameras=True, show_cameras=False)
runner = RobotRunner(robot, WiiUJoystick(), log_data=args.nolog, log_dir=log_dir, debug_prints=args.debug)

command_line = AutonomousCommandline()


def run_commands():
    command_line.cmdloop()
    print("Command line exiting")


t = Thread(target=run_commands)
# t.daemon = True
t.start()

runner.run()
