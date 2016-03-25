"""
Written by Ben Warwick, edited by Elim Zhang

road_edge_test.py, written for RoboQuasar3.0
Version 3/10/2015
=========

A test of the line following portion of the Self-Driving Buggy project.

Usage
-----
python __main__.py
- or - (in folder directory):
python Self-Driving\ Buggy\ Rev.\ 6

Keys
----
    q, ESC - exit
    SPACE - play/pause video
    o - toggle show original video feed
    left - read previous frame
    right - read next frame
    s - save frame as image (in camera/Images/ directory)
    v - start/stop create video (saved in camera/Videos/ directory)
    d - show average lines on screen
    a - show all lines found on screen within number tolerance
"""

import sys
import time

import cv2
import numpy as np

sys.path.insert(0, '../')

from vision import capture
from vision import analyzers


def run(paused=False,
        apply_filters=True,
        enable_draw=True,
        write_video=False,
        slideshow=False,
        burst_mode=False):
    camera1 = capture.Capture(
        window_name="line follow test",
        cam_source='Sat 27 Feb 2016 10;05;07 PM EST-1.avi',
        loop_video=False,
        start_frame=0,
        width=480,
        height=270
    )

    currentFrame = camera1.current_pos()

    frame1 = camera1.get_frame(readNextFrame=False)
    height, width = frame1.shape[0:2]

    time_start = time.time()

    warper = analyzers.RoadWarper(
        camera1.windowName, width, height,
        # [[135, 81], [355, 89], [4, 132], [478, 156]])
        [[189, 161], [347, 161], [5, 227], [474, 212]])
        # [[190, 110], [469, 110], [19, 160], [628, 160]])
    line_follower = analyzers.LineFollower()

    while camera1.isRunning:
        if not paused or currentFrame != camera1.current_pos():
            frame1 = camera1.get_frame()

            if frame1 is None:
                continue

            currentFrame = camera1.current_pos()

            if apply_filters:
                frame1 = warper.update(frame1)
                sobeled = cv2.cvtColor(frame1, cv2.COLOR_BGR2HSV)
                # sobeled = cv2.medianBlur(sobeled, 7)
                sobeled = cv2.GaussianBlur(sobeled, (5, 5), 0)
                sobeled = cv2.Sobel(sobeled, cv2.CV_64F, 1, 0, ksize=3)
                sobeled = np.absolute(sobeled)
                sobeled = np.uint8(sobeled)[:, :, 2]
                # sobeled = cv2.cvtColor(cv2.cvtColor(np.uint8(sobeled), cv2.COLOR_HSV2BGR),
                #                        cv2.COLOR_BGR2GRAY)
                # sobeled = np.uint8(sobeled)[:, :, 2]
                sobeled = cv2.adaptiveThreshold(sobeled, 255,
                                                cv2.ADAPTIVE_THRESH_MEAN_C,
                                                cv2.THRESH_BINARY, 13, -10)
                # value, sobeled = cv2.threshold(sobeled, 255, 255, cv2.THRESH_OTSU)
                # frame1 = cv2.inRange(sobeled, (70, ) * 3, (255, ) * 3)

                line_follower.update(sobeled, frame1)
                sobeled = cv2.cvtColor(np.uint8(sobeled), cv2.COLOR_GRAY2BGR)
                frame1 = np.concatenate((sobeled, frame1))

            if enable_draw:
                camera1.show_frame(frame1)

            if write_video:
                camera1.write_to_video(frame1)

        if slideshow:
            paused = True

        if burst_mode and paused:
            camera1.save_frame(frame1, default_name=True)

        if enable_draw:
            key = camera1.key_pressed()
            if key == 'q' or key == "esc":
                camera1.stop_camera()
            elif key == ' ':
                if paused:
                    print("%0.4fs, %i: ...Video unpaused" % (
                        time.time() - time_start, camera1.current_pos()))
                else:
                    print("%0.4fs, %i: Video paused..." % (
                        time.time() - time_start, camera1.current_pos()))
                paused = not paused
            elif key == 'o':
                apply_filters = not apply_filters
                print(("Applying filters is " + str(apply_filters)))
                frame1 = camera1.get_frame(False)
            elif key == "right":
                camera1.increment_frame()
            elif key == "left":
                camera1.decrement_frame()
            elif key == 's':
                camera1.save_frame(frame1)

            elif key == 'v':
                if not write_video:
                    camera1.start_video()
                else:
                    camera1.stop_video()
                write_video = not write_video
            elif key == 'b':  # burst photo mode
                burst_mode = not burst_mode
                print("Burst mode is " + str(burst_mode))
            elif key == 'p':  # debug print
                print("Frame #:", currentFrame)
            elif key == 'r':
                warper.reset()


if __name__ == '__main__':
    print(__doc__)
    run()
