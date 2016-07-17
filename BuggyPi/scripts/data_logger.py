import time
import sys

sys.path.insert(0, '../')

from robots.realbot import RealRobot
from robots.standard_config import *


def main():
    update_properties(
        log_data=True,
    )
    robot = RealRobot(properties, sensors, commands)

    while True:
        print(robot.get_state())
        time.sleep(0.05)
