'''
Written by Ben Warwick
With line analysis portion added by Elim Zhang

Self-Driving Buggy Rev. 6 for Self-Driving Buggy Project
Version 11/3/2015
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
'''

import sys
import cv2
import os
import numpy as np
import math

import time

sys.path.insert(0, '../')
from camera import capture
from camera import analyzers

def run():
    camera1 = capture.Capture(window_name="line follow test",
                              cam_source='Equinox 9-27 roll 2.mov',
                              loop_video=False,
                              start_frame=1386)

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
    # height, width = frame1.shape[0:2]
    
    time_start = time.time()
    
    while camera1.isRunning:
        if capture_properties['paused'] == False or capture_properties[
            'currentFrame'] != camera1.currentTimeMsec():
            frame1 = camera1.getFrame()
            
            if frame1 is None:
                continue

            capture_properties['currentFrame'] = camera1.currentTimeMsec()

            if capture_properties['apply_filters']:
                sobeled = cv2.medianBlur(frame1, 5)
                sobeled = cv2.Sobel(sobeled, cv2.CV_64F, 0, 1, ksize=3)
                sobeled = np.absolute(sobeled)
                frame1 = np.uint8(sobeled)
                # frame1 = cv2.inRange(sobeled, (70, ) * 3, (255, ) * 3)

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
                    print("%0.4fs, %i: ...Video unpaused" % (time.time() - time_start, camera1.currentTimeMsec()))
                else:
                    print("%0.4fs, %i: Video paused..." % (time.time() - time_start, camera1.currentTimeMsec()))
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
                    camera1.initVideoWriter()
                else:
                    camera1.stopVideo()
                capture_properties['write_video'] = not capture_properties[
                    'write_video']
            elif key == 'b':  # burst photo mode
                capture_properties['burst_mode'] = not capture_properties[
                    'burst_mode']
                print(("Burst mode is " + str(capture_properties['burst_mode'])))
            elif key == 'p':  # debug print
                print("Frame #:", capture_properties['currentFrame'])
    
if __name__ == '__main__':
    print(__doc__)
    run()
