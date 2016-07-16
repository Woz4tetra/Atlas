import cv2
import time
import os

import project

class Capture:
    def __init__(self, width, height, window_name, enable_draw):
        platform = project.get_platform()
        if platform == "linux":
            self.key_codes = {
                65362: "up",
                65364: "down",
                65361: "left",
                65363: "right",
                'esc': 'esc',
                10: "enter"
            }
        elif platform == "mac":
            self.key_codes = {
                63232: "up",
                63233: "down",
                63234: "left",
                63235: "right",
                13: "enter"
            }
        else:
            raise NotImplementedError

        self.width = width
        self.height = height

        self.window_name = window_name
        self.enable_draw = enable_draw

        self.frame = None
        
        self.video = None
        self.recorder_width, self.recorder_height = 0, 0
        self.recorder_output_dir = ""
        self.recording = False
        
        self.frame_num = 0
        self.slider_num = 0

    def get_frame(self):
        pass

    def set_frame(self, position):
        pass

    def current_pos(self):
        pass

    def increment_frame(self):
        pass

    def decrement_frame(self):
        pass

    def key_pressed(self, delay=1):
        key = cv2.waitKey(delay)
        if key in self.key_codes:
            return self.key_codes[key]
        elif key > -1:
            if 0 <= key < 0x100:
                return chr(key)
            else:
                print(("Unrecognized key: " + str(key)))
        else:
            return key

    def save_frame(self, frame=None, image_name=None, add_timestamp=True,
                   directory=None):
        """
        Write the input frame to Camera/Images

        :param frame: A numpy array containing the frame to write
                (shape = (height, width, 3))
        :return: None
        """

        if image_name is None:
            image_name = ""
        elif image_name is not None and add_timestamp:
            image_name += " "

        if add_timestamp:
            image_name += time.strftime("%c").replace(":", ";")

        if not image_name.endswith(".png"):
            image_name += ".png"

        print("Frame saved as " + str(image_name), end=" ")

        if directory is None:
            directory = project.get_dir(":images")
        print("in directory:\n" + directory)

        if not os.path.isdir(directory):
            os.makedirs(directory)
        if directory[-1] != "/":
            directory += "/"

        if frame is None:
            frame = self.frame
        
        cv2.imwrite(directory + image_name, frame)

    def show_frame(self, frame=None):
        """
        Display the frame in the Capture's window using cv2.imshow

        :param frame: A numpy array containing the image to be displayed
                (shape = (height, width, 3))
        :return: None
        """
        if self.enable_draw:
            if frame is not None:
                cv2.imshow(self.window_name, frame)
            else:
                cv2.imshow(self.window_name, self.frame)

    def start_recording(self, fps=32, video_name=None, add_timestamp=True,
                        output_dir=None, width=None, height=None,
                        with_frame=None):
        """
        Initialize the Capture's video writer.

        :param fps: The playback FPS of the video. This number can be finicky as
                the capture's current FPS may not match the video's output FPS.
                This is because video playback takes less computation than
                analyzing the video in this setting.
        :param video_name: The name of the video. If "", a time stamp will
                automatically be inserted
        :param add_timestamp: An optional parameter specifying whether the
                time should be included. True by default
        :param output_dir: directory to put video in

        :return: None
        """
        video_format = 'avi'
        codec = 'MJPG'

        if video_name is None:
            video_name = ""
        elif video_name is not None and add_timestamp:
            video_name += " "

        if add_timestamp:
            video_name += time.strftime("%c").replace(":", ";")

        video_name += "." + video_format

        if output_dir is None:
            output_dir = project.get_dir(":videos")
        else:
            output_dir += "/"

        if not os.path.isdir(output_dir):
            os.makedirs(output_dir)

        output_dir += video_name

        fourcc = cv2.VideoWriter_fourcc(*codec)
        self.video = cv2.VideoWriter()

        if width is None and with_frame is None:
            self.recorder_width = self.width
        else:
            if width is not None:
                self.recorder_width = width
            elif with_frame is not None:
                self.recorder_width = with_frame.shape[1]

        if height is None and with_frame is None:
            self.recorder_height = self.height
        else:
            if height is not None:
                self.recorder_height = height
            elif with_frame is not None:
                self.recorder_height = with_frame.shape[0]

        print(self.recorder_width, self.recorder_height)
        self.video.open(output_dir, fourcc, fps,
                        (self.recorder_width, self.recorder_height), True)
        self.recorder_output_dir = output_dir
        print("Initialized video named '%s'." % video_name)

        self.recording = True

    def record_frame(self, frame=None):
        """
            Write the frame to the Capture's initialized video capture.
            Type help(Capture.startVideo) for details.

            :param frame: A numpy array containing the frame to write
                    (shape = (height, width, 3))
            :return: None
            """
        if frame is None:
            frame = self.frame
        
        if frame.shape[0:2] != (self.recorder_height, self.recorder_width):
            frame = cv2.resize(frame,
                               (self.recorder_height, self.recorder_width))
        if len(frame.shape) == 2:
            self.video.write(cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR))
        else:
            self.video.write(frame)

    def stop_recording(self):
        """
            Close the initialized video capture.
            Type help(Capture.startVideo) for details.

            :return: None
            """
        if self.video is not None:
            self.video.release()
            print("Video written to:\n" + self.recorder_output_dir)

            self.video = None

            self.recording = False

    def stop(self):
        self.stop_recording()
        if self.enable_draw:
            cv2.destroyWindow(self.window_name)

