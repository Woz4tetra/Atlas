import sys
import time
import traceback
from pprint import pprint

sys.path.insert(0, "../")

from robots.autobot import AutoBot


def main():
    robot = AutoBot(log_data=False)

    try:
        while True:
            status = robot.update()
            pprint(robot.state)

            time.sleep(0.05)
            if not status:
                break
    except:
        traceback.print_exc()
    finally:
        robot.close()


if __name__ == '__main__':
    main()
