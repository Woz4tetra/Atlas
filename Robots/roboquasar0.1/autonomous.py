# This file is meant to collect data from manual control. Terminal interface only
import argparse

from atlasbuggy.interface import run

from joysticks.wiiu_joystick import WiiUJoystick
from roboquasar import RoboQuasar, map_sets

parser = argparse.ArgumentParser()
parser.add_argument("-l", "--nolog", help="disable logging", action="store_false")
parser.add_argument("-d", "--debug", help="enable debug prints", action="store_true")
args = parser.parse_args()

robot = RoboQuasar(True, False, False, *map_sets["cut"])

run(robot, WiiUJoystick(), log_data=args.nolog, log_dir=("data_days", None), debug_prints=args.debug)
