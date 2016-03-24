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

import numpy as np

sys.path.insert(0, '../')

from camera import capture
from camera import analyzers
from camera import cv_pid

def run(paused=False, apply_filters=True, enable_draw=True, write_video=False):
    camera1 = capture.Capture(
        window_name="line follow test",
        cam_source='Sat 27 Feb 2016 10;05;07 PM EST-1.avi',
        loop_video=False,
        start_frame=1500,
        # width=480,
        # height=270,
        crop=[None, 135, None, None],
    )

    current_frame = camera1.current_pos()

    frame1 = camera1.get_frame(readNextFrame=False)
    height, width = frame1.shape[0:2]

    time_start = time.time()

    if write_video:
        camera1.start_video(width=width, height=height * 2, format='avi')

    pid = cv_pid.PID(width / 2, 0.1, 0, 0)

    while camera1.isRunning:
        if not paused or current_frame != camera1.current_pos():
            frame1 = camera1.get_frame()

            if frame1 is None:
                continue

            current_frame = camera1.current_pos()

            if apply_filters:
                lines, detected = analyzers.detect_lines(frame1)
                if enable_draw:
                    analyzers.draw_lines(frame1, lines)

                servo_angle = pid.update(
                    analyzers.get_pid_goal(lines, 240, 480, frame1))
                print(servo_angle)

                frame1 = np.concatenate((detected, frame1))

            if enable_draw:
                camera1.show_frame(frame1)

            if write_video:
                camera1.write_to_video(frame1)

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
            elif key == 'p':  # debug print
                print("Frame #:", current_frame)


if __name__ == '__main__':
    print(__doc__)
    run()
