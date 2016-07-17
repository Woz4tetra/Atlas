import sys
import time
import traceback
from pprint import pprint

sys.path.insert(0, "../")

from robots.auto_config import AutoBot


def main():
    robot = AutoBot(log_data=False)

    goal_x = robot.checkpoints[2][0]
    goal_y = robot.checkpoints[2][1]
    print(goal_x, goal_y)
    try:
        while True:
            robot.update_filter()
            speed, servo_value = robot.controller.update(
                robot.state, goal_x, goal_y
            )
            print("%0.4f\t%i, (%0.6f, %0.6f), (%0.6f, %0.6f)" % (speed, servo_value, goal_x - robot.state["x"], goal_y - robot.state["y"], robot.state["x"], robot.state["y"]))
            robot.motors.set(int(speed * 100))
            robot.blue_led.set(int(speed * 255 / 100))
            robot.servo.set(servo_value)
##            if not status:
##                break
    except:
        traceback.print_exc()
    finally:
        robot.close()


if __name__ == '__main__':
    main()
