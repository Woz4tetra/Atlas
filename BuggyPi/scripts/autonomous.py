import time
import sys

sys.path.insert(0, '../')

from robots.realbot import RealRobot
from robots.standard_config import *


def main():
    bearing = get_gps_bearing(
        -71.420864, 42.427317, -71.420795, 42.427332
    )
    bearing = (-bearing - math.pi / 2) % (2 * math.pi) + 0.1
    update_properties(
        initial_long="gps",
        initial_lat="gps",
        initial_heading=bearing
    )
    robot = RealRobot(properties, sensors, commands)

    while True:
        print(robot.get_state())

        # control stuff...

        time.sleep(0.05)
