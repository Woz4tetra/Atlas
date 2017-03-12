from atlasbuggy.files.videofile import *
from atlasbuggy import get_platform
import numpy as np


class Capture:
    def __init__(self, name, width=None, height=None, enable_show=True, enable_recording=True):
        self.reader = None
        self.writer = None
        self.capture = None

        self.enable_show = enable_show
        self.name = name
        if self.enable_show:
            cv2.namedWindow(self.name)

        self.width = width
        self.height = height

        self.enable_recording = enable_recording

        platform = get_platform()
        if platform == "linux":
            self.key_codes = {
                65362: "up",
                65364: "down",
                65361: "left",
                65363: "right",
                10   : "enter"
            }
        elif platform == "mac":
            self.key_codes = {
                63232: "up",
                63233: "down",
                63234: "left",
                63235: "right",
                10   : "enter"
            }
        else:
            self.key_codes = {}

    def play_video(self, video_name=None, video_dir=None):
        self.reader = VideoPlayer(video_name, video_dir, self.name, self, self.width, self.height)
        self.capture = self.reader.cv_capture

    def start_camera(self, video_name=None, video_dir=None, camera_number=None):
        if camera_number is None:
            selector = CameraSelector(self.name)
            selector.launch()
            if selector.cv_capture is None:
                return "done"
            else:
                camera_number = selector.capture_num
        self.writer = VideoRecorder(video_name, video_dir, self.width, self.height, self.enable_recording, self,
                                    camera_number, selector.cv_capture)
        self.capture = self.writer.cv_capture

    def show_frame(self, frame=None):
        """
        Display the frame in the Capture's window using cv2.imshow

        :param frame: A numpy array containing the image to be displayed
                (shape = (height, width, 3))
        :return: None
        """
        if frame is not None:
            show_frame = frame
        elif self.reader is not None:
            show_frame = self.reader.frame
        else:
            show_frame = self.writer.frame

        cv2.imshow(self.name, show_frame)

        return self.key_pressed()

    def get_frame(self, dt):
        if self.reader is not None:
            return self.reader.get_frame(dt)
        else:
            return self.writer.get_frame()

    def close(self):
        if self.writer is not None:
            self.writer.close()
        if self.enable_show:
            cv2.destroyWindow(self.name)

    def key_pressed(self, delay=1):
        key = cv2.waitKey(delay)
        if key in self.key_codes:
            return self.key_codes[key]
        if key > -1:
            if 0 <= key < 0x100:
                return chr(key)
            else:
                print(("Unrecognized key: " + str(key)))
        else:
            return key

class CameraSelector(Capture):
    def __init__(self, name):
        super(CameraSelector, self).__init__("Camera Selector: " + name, 720, 480, enable_recording=False)

    def launch(self):
