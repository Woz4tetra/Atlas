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

from microcontroller.data import *
from microcontroller.dashboard import *

from controllers.servo_map import angle_to_servo

from vision.usb_cameras import Logitech
from vision import analyzers
from vision import cv_pid

from sound.player import TunePlayer

def run(paused=False, apply_filters=True, enable_draw=True, write_video=False,
        night_mode=False, crop=(135, 350)):
    camera1 = Logitech(fps_size=30, cam_source=1)

    frame1 = camera1.get_frame()
    height, width = frame1.shape[0:2]

    time_start = time.time()

    notifier = TunePlayer()

    if write_video:
        camera1.start_recording(width=width, height=height, format='avi')

    pid = cv_pid.PID(width / 2, 0.1, 0, 0)

    servo_steering = Command(0, 'position', (-90, 90))

    start(use_handshake=False)
    reset()

    notifier.play("ding")

    while camera1.is_running:
        if not paused:
            frame1 = camera1.get_frame()

            if frame1 is None:
                continue

            if apply_filters:
                frame1 = frame1[crop[0]: crop[1]]
                frame1 = frame1[135:]
                lines, detected = analyzers.detect_lines(frame1, night_mode)
                if enable_draw:
                    analyzers.draw_lines(frame1, lines)

                servo_angle = pid.update(
                    analyzers.get_pid_goal(lines, width // 2, height, frame1))
                print(servo_angle)

                servo_steering["position"] = angle_to_servo(servo_angle)

                frame1 = np.concatenate((detected, frame1))

            if enable_draw:
                camera1.show_frame(frame1)

            if write_video:
                camera1.record_frame(frame1)

        if enable_draw:
            key = camera1.key_pressed()
            if key == 'q' or key == "esc":
                camera1.stop()
            elif key == ' ':
                if paused:
                    print("%0.4fs: ...Video unpaused" % (
                        time.time() - time_start))
                else:
                    print("%0.4fs: Video paused..." % (
                        time.time() - time_start))
                paused = not paused
            elif key == 'o':
                apply_filters = not apply_filters
                print(("Applying filters is " + str(apply_filters)))
                frame1 = camera1.get_frame()
            elif key == 's':
                camera1.save_frame(frame1)

            elif key == 'v':
                if not write_video:
                    print(frame1.shape)
                    camera1.start_recording(with_frame=frame1, format='avi')
                else:
                    camera1.stop_video()
                write_video = not write_video


if __name__ == '__main__':
    print(__doc__)
    run()
