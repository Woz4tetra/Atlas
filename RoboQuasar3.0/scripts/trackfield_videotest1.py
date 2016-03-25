
"""
Written by Ben Warwick

trackfield_videotest1.py, written for RoboQuasar3.0
Version 2/29/2015
=========

Video recorder. Written for test day 1
"""

import sys
import cv2
import os
import numpy as np
import math

import time

sys.path.insert(0, '../')

from vision import capture
from vision import analyzers


def run():
    camera1 = capture.Capture(window_name="camera1",
        cam_source=1,
        width=640,
        height=480
    )

    camera2 = capture.Capture(window_name="camera2",
        cam_source=2,
        width=640,
        height=480
    )

    capture_properties = dict(
            paused=False,
            apply_filters=False,
            enable_draw=True,
            currentFrame=camera1.current_pos(),
            write_video=False,
            slideshow=False,
            burst_mode=False,
    )

    # frame1 = camera1.getFrame(readNextFrame=False)
    # height, width = frame1.shape[0:2]

    time_start = time.time()

    # warper = analyzers.RoadWarper(camera1.windowName, width, height,
    #                               [[209, 116], [510, 116], [12, 203], [631, 203]])
                                #   [[190, 110], [469, 110], [19, 160], [628, 160]])

    while camera1.isRunning:
        if capture_properties['paused'] == False or capture_properties[
                'currentFrame'] != camera1.current_pos():
            frame1 = camera1.get_frame()
            frame2 = camera2.get_frame()

            if frame1 is None or frame2 is None:
                continue

            capture_properties['currentFrame'] = camera1.current_pos()

            if capture_properties['apply_filters']:
                # sobeled = warper.update(frame1)
                sobeled = cv2.cvtColor(frame1, cv2.COLOR_BGR2HSV)
                # sobeled = cv2.medianBlur(sobeled, 7)
                sobeled = cv2.GaussianBlur(sobeled, (5, 5), 0)
                sobeled = cv2.Sobel(sobeled, cv2.CV_64F, 1, 0, ksize=3)
                sobeled = np.absolute(sobeled)
                sobeled = np.uint8(sobeled)[:, :, 2]
                # sobeled = cv2.cvtColor(cv2.cvtColor(np.uint8(sobeled), cv2.COLOR_HSV2BGR),
                #                        cv2.COLOR_BGR2GRAY)
                # sobeled = np.uint8(sobeled)[:, :, 2]
                value, frame1 = cv2.threshold(sobeled, 255, 255, cv2.THRESH_OTSU)
                # frame1 = cv2.inRange(sobeled, (70, ) * 3, (255, ) * 3)


            if capture_properties['enable_draw'] is True:
                camera1.show_frame(frame1)
                camera2.show_frame(frame2)

            if capture_properties['write_video'] == True:
                camera1.write_to_video(frame1)
                camera2.write_to_video(frame2)

        if capture_properties['slideshow'] == True:
            capture_properties['paused'] = True

        if capture_properties['burst_mode'] == True and capture_properties[
            'paused'] == False:
            camera1.save_frame(frame1, default_name=True)
            camera2.save_frame(frame2, default_name=True)

        if capture_properties['enable_draw'] is True:
            key = camera1.key_pressed()
            if key == 'q' or key == "esc":
                camera1.stop_camera()
                camera2.stop_camera()
            elif key == ' ':
                if capture_properties['paused']:
                    print("%0.4fs, %i, %i: ...Video unpaused" % (
                        time.time() - time_start, camera1.current_pos(),
                        camera2.current_pos()))
                else:
                    print("%0.4fs, %i: Video paused..." % (
                        time.time() - time_start, camera1.current_pos(),
                        camera2.current_pos()))
                capture_properties['paused'] = not capture_properties['paused']
            elif key == 'o':
                capture_properties['apply_filters'] = not capture_properties[
                    'apply_filters']
                print((
                    "Applying filters is " + str(
                            capture_properties['apply_filters'])))
                frame1 = camera1.get_frame(False)
                frame2 = camera2.get_frame(False)
            elif key == 's':
                camera1.save_frame(frame1)
                camera2.save_frame(frame2)

            elif key == 'v':
                if capture_properties['write_video'] == False:
                    camera1.start_video()
                    camera2.start_video()
                else:
                    camera1.stop_video()
                    camera2.stop_video()
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
