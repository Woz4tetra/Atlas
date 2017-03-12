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
                13   : "enter"
            }
        else:
            self.key_codes = {}

    def play_video(self, video_name=None, video_dir=None):
        self.reader = VideoPlayer(video_name, video_dir, self.name, self, self.width, self.height)
        self.capture = self.reader.cv_capture

    def start_camera(self, video_name=None, video_dir=None, camera_number=None):
        if camera_number is None:
            selector = CameraSelector()
            selector.launch()
            if selector.cv_capture is None:
                return "done"
            else:
                camera_number = selector.capture_num
        self.writer = VideoRecorder(video_name, video_dir, self.width, self.height, self.enable_recording, self,
                                    camera_number)
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


class CameraSelector:
    def __init__(self):
        print("Welcome to the Camera Selector!\n\n")
        print("Use the arrow keys to switch between cameras.")
        print("Enter numbers on your number pad to jump to different cameras.")
        print("Press enter to confirm and q or esc to cancel.")

        self.shape = None
        self.window_name = "Camera Selector"
        self.capture_num = 0
        self.captures = {}

        self.cv_capture = cv2.VideoCapture(self.capture_num)
        self.captures[self.capture_num] = self.cv_capture

        self.cv_capture.set(cv2.CAP_PROP_FRAME_WIDTH, 720)
        self.cv_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

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
                13   : "enter"
            }
        else:
            self.key_codes = {}

    def launch(self):
        while True:
            key = self.key_pressed()
            if key == "left":
                print("Loading camera. Please wait...")
                self.update_capture(-1)
            elif key == "right":
                print("Loading camera. Please wait...")
                self.update_capture(1)
            elif type(key) == str and key.isdigit():
                self.update_capture(new_capture=int(key))
            elif key == "enter":
                cv2.destroyWindow(self.window_name + str(self.capture_num))
            elif key == 'q' or key == "esc":
                self.cv_capture = None
                return

            success, frame = self.cv_capture.read()

            if success is True and frame is not None:
                cv2.imshow(self.window_name, frame)
                self.shape = frame.shape
            else:
                cv2.imshow(self.window_name, np.zeros(self.shape))

    def update_capture(self, delta=None, new_capture=None):
        # self.cv_capture.release()
        # cv2.destroyWindow(self.window_name + str(self.capture_num))

        if delta is not None:
            self.capture_num += delta
        elif new_capture is not None:
            self.capture_num = new_capture

        if self.capture_num in self.captures:
            self.cv_capture = self.captures[self.capture_num]
        else:
            self.cv_capture = cv2.VideoCapture(self.capture_num)
            self.cv_capture.set(cv2.CAP_PROP_FRAME_WIDTH, 720)
            self.cv_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

            self.captures[self.capture_num] = self.cv_capture

        print(self.capture_num)

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
