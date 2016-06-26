##import sys
import os
import time
import cv2

from picamera.array import PiRGBArray
from picamera import PiCamera

##sys.path.insert(0, '../')

from vision.capture import Capture
import directories

class Camera(Capture):
    def __init__(self, width, height, window_name="PiCamera", enable_draw=True, **args):
        super(Camera, self).__init__(width, height, window_name, enable_draw)
            
        self.camera = PiCamera(**args)
        self.camera.resolution = self.width, self.height
        self.raw_capture = PiRGBArray(self.camera, size=(self.width, self.height))
        time.sleep(0.1)

        self.capture = self.camera.capture_continuous(self.raw_capture, format="bgr", use_video_port=True)
        self.frame_num = 0

    def get_frame(self):
        self.frame = next(self.capture).array

        # clear the stream in preparation for the next frame
        self.raw_capture.truncate(0)

        self.frame_num += 1
        
        return self.frame

    def current_pos(self):
        return self.frame_num
