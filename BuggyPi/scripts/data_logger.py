import time
import sys

sys.path.insert(0, '../')

from robots.realbot import RealRobot
from robots.standard_config import *


def main():
    bearing = get_gps_bearing(
        -71.420864, 42.427317, -71.420795, 42.427332
    )
    bearing = (-bearing - math.pi / 2) % (2 * math.pi)
    update_properties(
        initial_long="gps",
        initial_lat="gps",
        initial_heading=bearing,
        log_data=True
    )
    robot = RealRobot(properties, sensors, commands)

    try:
        while True:
            print(robot)
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass

    robot.close()
