import sys
import time
from threading import Thread
import cv2

from picamera import PiCamera
from picamera.array import PiRGBArray

sys.path.insert(0, '../')

from vision.capture import Capture


class Camera(Capture):
    def __init__(self, width, height, window_name="PiCamera", enable_draw=True,
                 pipeline=None, **args):
        super(Camera, self).__init__(width, height, window_name, enable_draw)

        self.camera = PiCamera(**args)
        self.camera.resolution = self.width, self.height
        self.raw_capture = PiRGBArray(self.camera,
                                      size=(self.width, self.height))
        time.sleep(0.1)

        self.capture = self.camera.capture_continuous(
            self.raw_capture, format="bgr", use_video_port=True
        )
        self.stopped = False

        self.pipeline = pipeline

        self.analyzed_frame = None
        self.pipeline_results = {}

    def start(self):
        # start the thread to read frames from the video stream
        Thread(target=self.update, args=()).start()
        return self

    def stop(self):
        self.stop_recording()
        if self.enable_draw:
            cv2.destroyWindow(self.window_name)

        # indicate that the thread should be stopped
        self.stopped = True

    def update(self):
        # keep looping infinitely until the thread is stopped
        for f in self.capture:
            # grab the frame from the stream and clear the stream in
            # preparation for the next frame
            self.frame = f.array
            self.raw_capture.truncate(0)

            if self.pipeline is not None:
                self.analyzed_frame, self.pipeline_results = \
                    self.pipeline.update(self, self.frame)

            # if the thread indicator variable is set, stop the thread
            # and resource camera resources
            if self.stopped:
                self.capture.close()
                self.raw_capture.close()
                self.camera.close()
                return

            if self.capture.recording:
                self.capture.record_frame()

            self.frame_num += 1
            self.slider_num += 1

    def get_frame(self):
        return self.frame

    def current_pos(self):
        return self.frame_num
