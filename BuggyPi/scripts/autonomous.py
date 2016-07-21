import sys
import traceback

sys.path.insert(0, '../')

from robots.realbot import RealRobot
from robots.standard_config import *
from navigation.controller import Controller


def main():
    bearing = get_gps_bearing(
        -71.420864, 42.427317, -71.420795, 42.427332
    )
    bearing = (-bearing - math.pi / 2) % (2 * math.pi)
    update_properties(
        initial_long="gps",
        initial_lat="gps",
        initial_heading=bearing,
        enable_camera=False,
        goal_point=2
    )
    robot = RealRobot(properties, sensors, commands)
    robot.add_property('pid', Controller(
##        1, 0, 0, robot.left_angle_limit, robot.right_angle_limit,
        25000, 0.0, 0.0, 0.0, 1.0))
    goal_x, goal_y = robot.checkpoints[robot.goal_point]
    robot.add_property('goal_x', goal_x)
    robot.add_property('goal_y', goal_y)
    try:
        while True:
            # print(robot)

            if not robot.manual_mode:
                angle_command, speed_command = \
                    robot.pid.update(robot.get_state(), robot.goal_x, robot.goal_y)
                if 0.08 < speed_command < 0.4:
                    speed_command = 0.4
                elif speed_command <= 0.08:
                    speed_command = 0.0

                print(robot.angle_to_servo(angle_command), speed_command)
                state = robot.get_state()
                print(("%0.7f, " * 3) % (state["x"], state["y"], state["angle"]))
                print(("%0.7f, " * 3) % (robot.goal_x, robot.goal_y, angle_command))
                robot.servo.set(robot.angle_to_servo(angle_command))
                robot.motors.set(int(speed_command * 100))
                robot.blue_led.set(int(speed_command * 255))
                
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
