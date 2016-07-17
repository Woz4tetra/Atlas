import time
import sys

sys.path.insert(0, '../')

from robots.realbot import RealRobot
from robots.standard_config import *
from navigation.pd_controller import Controller

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
    pid = Controller(0.001, robot.front_back_dist, 1000, 1,
                     robot.left_angle_limit, robot.right_angle_limit,
                     robot.left_servo_limit, robot.right_servo_limit)
    goal_x, goal_y = robot.checkpoints[2]
    try:
        while True:
            print(robot)
            robot.update_camera()

            speed_command, servo_command = \
                pid.update(robot.get_state(), goal_x, goal_y)
            robot.servo.set(servo_command)
            robot.motor.set(int(speed_command))
            robot.blue_led.set(int(speed_command))

            time.sleep(0.05)
    except:
        robot.close()

if __name__ == '__main__':
    main()
