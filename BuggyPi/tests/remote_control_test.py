#from subprocess import call
#call("raspivid -n -hf -w 1280 -h 1024 -t 999999999 -fps 30 -b 5000000 -o - | nc 192.168.1.91 5001")
import sys
import time
import traceback

sys.path.insert(0, "../")

from robots.loggerbot import LoggerBot


def main():
    robot = LoggerBot(log_data=False)
    for _ in range(8):
        robot.leds["green"].set("toggle")
        time.sleep(0.05)

    try:
        while True:
            status = robot.update()
            if not status:
                break
    except:
        traceback.print_exc()
    finally:
        robot.close()