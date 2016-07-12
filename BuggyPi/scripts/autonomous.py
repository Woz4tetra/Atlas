import sys
import traceback

sys.path.insert(0, "../")

from robots.autobot import AutoBot


def main():
    robot = AutoBot()

    try:
        while True:
            status = robot.update()
            if not status:
                break
    except:
        traceback.print_exc()
    finally:
        robot.close()


if __name__ == '__main__':
    main()
