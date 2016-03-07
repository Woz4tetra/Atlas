"""
The minimum code required to run the camera module
"""
import sys

sys.path.insert(0, '../')

import config
from camera import capture


def run():
    camera1 = capture.Capture(window_name="camera", cam_source=0)
    while True:
        frame1 = camera1.getFrame()
        camera1.showFrame(frame1)
        key = camera1.getPressedKey()
        if key != -1:
            print(repr(key))


if __name__ == '__main__':
    run()
