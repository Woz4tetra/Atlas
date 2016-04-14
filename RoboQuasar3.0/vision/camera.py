import time

import cv2
import numpy as np

from vision.capture import Capture


class Camera(Capture):
    def __init__(self, width, height, window_name=None, enable_draw=True, cam_source=None):
        time0 = time.time()

        super(Camera, self).__init__(window_name, width, height, enable_draw)
        self.cam_source = cam_source

        if window_name is None:
            window_name = "camera " + str(Camera.capture_num)

        if self.enable_draw:
            print(window_name)
            cv2.namedWindow(window_name)

        if cam_source is not None:
            self.capture = self.load_capture(cam_source)
        else:
            self.capture = self.camera_selector()

        time1 = time.time()
        print((str(self.cam_source) + " loaded in " + str(
            time1 - time0) + " seconds. Capture size is " +
               str(int(self.width)) + "x" + str(int(self.height))))

    def get_frame(self):
        if not self.is_running:
            self.stop()
            return None
        success, self.frame = self.capture.read()

        if success is False or self.frame is None:
            raise IOError("Failed to read from camera!")

        return self.frame

    def load_capture(self, capture):
        """
            Loads a numbered camera and adds it to Capture.excludedSources so that
            duplicate cameras don't arise.

            :param capture: The camera number to load
            :return: None
            """
        print("loading camera " + str(capture) + " into window named '" + str(
            self.window_name) + "'...")
        return cv2.VideoCapture(capture)

    def camera_selector(self):
        """
            A mini application to assist in camera selection. Cycle through the
            cameras with the left and right arrow keys and jump between cameras with
            the number pad. Press enter to confirm and q or esc to cancel.

            :return: The selected camera number
            """
        print("Welcome to the Camera Selector!\n\n")
        print("Use the arrow keys to switch between cameras.")
        print("Enter numbers on your number pad to jump to different cameras.")
        print("Press enter to confirm and q or esc to cancel.")

        shape = None
        window_name = "Camera Selector: camera #"
        capture_num = 0

        def update_capture(window_name, video, capture_num, delta=None,
                           new_capture=None):
            video.release()

            cv2.destroyWindow(window_name + str(capture_num))
            if delta is not None:
                capture_num += delta
            elif new_capture is not None:
                capture_num = new_capture
            print(capture_num)

            video = cv2.VideoCapture(capture_num)

            video.set(cv2.CAP_PROP_FRAME_WIDTH, 720)
            video.set(cv2.CAP_PROP_FRAME_HEIGHT, 450)

            return video, capture_num

        temp_capture = cv2.VideoCapture(capture_num)

        temp_capture.set(cv2.CAP_PROP_FRAME_WIDTH, 720)
        temp_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 450)

        while True:
            key = self.key_pressed(1)
            if key == "left":
                print("Loading camera. Please wait...")
                temp_capture, capture_num = update_capture(window_name,
                                                       temp_capture, capture_num,
                                                       delta=-1)
            elif key == "right":
                print("Loading camera. Please wait...")
                temp_capture, capture_num = update_capture(window_name,
                                                       temp_capture, capture_num,
                                                       delta=+1)
            elif type(key) == str and key.isdigit():
                temp_capture, capture_num = update_capture(window_name,
                                                           temp_capture, capture_num,
                                                           new_capture=int(key))
            elif key == "enter":
                cv2.destroyWindow(window_name + str(capture_num))
                return temp_capture
            elif key == 'q' or key == "esc":
                quit()

            success, frame = temp_capture.read()

            if success is True and frame is not None:
                cv2.imshow(window_name + str(capture_num), frame)
                shape = frame.shape
            else:
                cv2.imshow(window_name + str(capture_num), np.zeros(shape))
