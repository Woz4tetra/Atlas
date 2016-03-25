"""
Written by Ben Warwick

diagnose_system.py, written for RoboQuasar3.0
Version 3/10/2015
=========

The minimum code required to run the camera module.
Prints the key ID of the pressed keyboard button
"""
import sys

sys.path.insert(0, '../')

import config
from vision.camera import Camera


def run():
    camera1 = Camera(480, 360, window_name="camera")
    while True:
        frame1 = camera1.get_frame()
        camera1.show_frame(frame1)
        key = camera1.key_pressed()
        if key != -1:
            print(repr(key))


if __name__ == '__main__':
    run()