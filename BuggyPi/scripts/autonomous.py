import sys
import traceback

sys.path.insert(0, '../')

from robots.realbot import RealRobot
from robots.standard_config import *
from navigation.pd_controller import Controller


def main():
    bearing = get_gps_bearing(
        -71.420864, 42.427317, -71.420795, 42.427332
    )
    bearing = (-bearing - math.pi / 2) % (2 * math.pi)
    update_properties(
        initial_long="gps",
        initial_lat="gps",
        initial_heading=bearing
    )
    robot = RealRobot(properties, sensors, commands)
    pid = Controller(1, 1, 1,
                     robot.left_angle_limit, robot.right_angle_limit,
                     1, 1, 1, 0, 100)
    goal_x, goal_y = robot.checkpoints[2]
    try:
        while True:
            # print(robot)

            if not robot.manual_mode:
                speed_command, angle_command = \
                    pid.update(robot.get_state(), goal_x, goal_y)
                robot.servo.set(robot.angle_to_servo(angle_command))
                robot.motors.set(int(speed_command))
                robot.blue_led.set(int(speed_command))
                print(speed_command, angle_command)
                time.sleep(0.05)

            if not robot.camera_running:
                print("Camera signaled to stop...")
                break

    except:
        traceback.print_exc()
    finally:
        robot.close()


if __name__ == '__main__':
    main()
