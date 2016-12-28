import cv2
from atlasbuggy.vision.base_camera import BaseCamera


class Camera(BaseCamera):
    """A class for reading from any camera that opencv recognizes"""

    def __init__(self, width=None, height=None, preset=None,
                 window_name="Camera", cam_source=None, enable_draw=True,
                 pipeline=None, update_fn=None, fn_params=None,
                 resolutions=None):
        """
        :param width: set a width for the capture
        :param height: set a height for the capture
        :param window_name: set a opencv window name
        :param enable_draw: whether the opencv window should be shown
            (boosts frames per second)
        :param pipeline: a class with a method named update. This class should
            parse the frame a return any useful data
        :param update_fn: the camera runs on a separate thread. Put any extra
            code to run in this function
        :param fn_params: parameters to pass to update_fn
        """

        if width is None and height is None and resolutions is None:
            raise ValueError("Please provide a width and height specification "
                             "with either fps_size and resolutions dictionary "
                             "or with a width and height value.")

        super(Camera, self).__init__(width, height, window_name, enable_draw,
                                     pipeline, update_fn, fn_params)
        self.resolutions = resolutions
        self.cam_source = cam_source

        if self.width is None and self.height is None:
            if preset < len(self.resolutions):
                self.width, self.height = self.resolutions[preset]
            else:
                self.width, self.height = self.resolutions[None]

        if self.window_name is None:
            self.window_name = "Camera " + str(self.cam_source)

        if self.cam_source is not None:
            self.camera = self.load_capture(self.cam_source)
        else:
            self.camera, self.cam_source = self.camera_selector(
                self.width, self.height)

    def load_capture(self, capture):
        """
            Loads a numbered camera and adds it to Capture.excludedSources so that
            duplicate cameras don't arise.

            :param capture: The camera number to load
            :return: None
            """
        print("loading camera " + str(capture) + " into window named '" + str(
            self.window_name) + "'...")
        camera = cv2.VideoCapture(capture)

        camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        return camera

    def camera_selector(self, width, height):
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

        temp_capture = cv2.VideoCapture(capture_num)

        temp_capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        temp_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        def update_capture(wind_name, video, cap_num, delta=None,
                           new_capture=None):
            video.release()

            cv2.destroyWindow(wind_name + str(cap_num))
            if delta is not None:
                cap_num += delta
            elif new_capture is not None:
                cap_num = new_capture
            print(cap_num)

            video = cv2.VideoCapture(cap_num)

            video.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            video.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

            return video, cap_num

        while True:
            key = self.key_pressed(1)
            if key == "left":
                print("Loading camera. Please wait...")
                temp_capture, capture_num = update_capture(
                    window_name, temp_capture, capture_num, delta=-1)
            elif key == "right":
                print("Loading camera. Please wait...")
                temp_capture, capture_num = update_capture(
                    window_name, temp_capture, capture_num, delta=+1)
            elif type(key) == str and key.isdigit():
                temp_capture, capture_num = update_capture(
                    window_name, temp_capture, capture_num,
                    new_capture=int(key))
            elif key == "enter":
                cv2.destroyWindow(window_name + str(capture_num))
                return temp_capture, capture_num
            elif key == 'q' or key == "esc":
                quit()

            success, frame = temp_capture.read()

            if success is True and frame is not None:
                cv2.imshow(window_name + str(capture_num), frame)
                shape = frame.shape
            else:
                cv2.imshow(window_name + str(capture_num), np.zeros(shape))

    def update(self):
        """Keep reading from the camera until self.stopped is True"""
        while True:
            success, self.frame = self.camera.read()
            self.key_pressed()

            if success is False or self.frame is None:
                print("Failed to read from camera!")
                self.stopped = True

            if self.stopped:
                # self.camera.release()
                break

            if self.pipeline is not None:
                self.analyzed_frame, self.pipeline_results = \
                    self.pipeline.update(self, self.frame)

            if self.is_recording:
                self.record_frame()

            if self.update_fn is not None:
                if not self.update_fn(self.fn_params):
                    self.stop()

            self.frame_num += 1
            self.slider_num += 1


if __name__ == '__main__':
    import traceback
    test_camera_module = Camera(400, 300, cam_source=0)
    test_camera_module.start()

    try:
        while True:
            test_camera_module.key_pressed()
            test_camera_module.show_frame()
    except:
        print("stopping")
        test_camera_module.stop()
        traceback.print_exc()
