
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
import cv2
import os
import numpy as np
import math

import time

sys.path.insert(0, '../')

from camera import capture
from camera import analyzers

import copy
def create_array(value, width, height):
    return [[copy.copy(value) for _ in range(width)] for _ in range(height)]

def run():
    camera1 = capture.Capture(window_name="line follow test",
        cam_source='Icarus 10-11 roll 5 (+hill 1).mov',
        # cam_source='',
        loop_video=False,
        start_frame=0,
        width=480,
        height=270
    )

    capture_properties = dict(
            paused=False,
            apply_filters=True,
            enable_draw=True,
            currentFrame=camera1.currentTimeMsec(),
            write_video=False,
            slideshow=False,
            burst_mode=False,
    )

    frame1 = camera1.getFrame(readNextFrame=False)
    height, width = frame1.shape[0:2]

    time_start = time.time()

    warper = analyzers.RoadWarper(camera1.windowName, width, height,
                                  [[135, 81], [355, 89], [4, 132], [478, 156]])
                                #   [[190, 110], [469, 110], [19, 160], [628, 160]])
    line_follower = analyzers.LineFollower()

    while camera1.isRunning:
        if capture_properties['paused'] == False or capture_properties[
                'currentFrame'] != camera1.currentTimeMsec():
            frame1 = camera1.getFrame()

            if frame1 is None:
                continue

            capture_properties['currentFrame'] = camera1.currentTimeMsec()

            if capture_properties['apply_filters']:
                frame1 = warper.update(frame1)
                factors = 4
                sums = create_array(0, 1, 4)
                print(sums)
                for factor in range(0, factors):
                    for x in range(0,len(frame1),10):
                        sums[factor].append(0)
                        for y in range(len(frame1[x])):
                            for c in range(3):
                                sums[factor][x // 10] += frame1[x][y//(factor + 1)][c]
                        # print (sums[factor][x // 10])
                    print("something")

                # print(sums)

                deltas = [0] * factors
                for factor in range(factors):
                    for x in range(1,len(sums[factor])):
                        deltas[factor] += abs(sums[factor][x] - sums[factor][x-1])
                print(deltas)

            if capture_properties['enable_draw'] is True:
                camera1.showFrame(frame1)

            if capture_properties['write_video'] == True:
                camera1.writeToVideo(frame1)

        if capture_properties['slideshow'] == True:
            capture_properties['paused'] = True

        if capture_properties['burst_mode'] == True and capture_properties[
            'paused'] == False:
            camera1.saveFrame(frame1, default_name=True)

        if capture_properties['enable_draw'] is True:
            key = camera1.getPressedKey()
            if key == 'q' or key == "esc":
                camera1.stopCamera()
            elif key == ' ':
                if capture_properties['paused']:
                    print("%0.4fs, %i: ...Video unpaused" % (
                    time.time() - time_start, camera1.currentTimeMsec()))
                else:
                    print("%0.4fs, %i: Video paused..." % (
                    time.time() - time_start, camera1.currentTimeMsec()))
                capture_properties['paused'] = not capture_properties['paused']
            elif key == 'o':
                capture_properties['apply_filters'] = not capture_properties[
                    'apply_filters']
                print((
                    "Applying filters is " + str(
                            capture_properties['apply_filters'])))
                frame1 = camera1.getFrame(False)
            elif key == "right":
                camera1.incrementFrame()
            elif key == "left":
                camera1.decrementFrame()
            elif key == 's':
                camera1.saveFrame(frame1)

            elif key == 'v':
                if capture_properties['write_video'] == False:
                    camera1.startVideo()
                else:
                    camera1.stopVideo()
                capture_properties['write_video'] = not capture_properties[
                    'write_video']
            elif key == 'b':  # burst photo mode
                capture_properties['burst_mode'] = not capture_properties[
                    'burst_mode']
                print((
                      "Burst mode is " + str(capture_properties['burst_mode'])))
            elif key == 'p':  # debug print
                print("Frame #:", capture_properties['currentFrame'])
            elif key == 'r':
                warper.reset()


if __name__ == '__main__':
    print(__doc__)
    run()
