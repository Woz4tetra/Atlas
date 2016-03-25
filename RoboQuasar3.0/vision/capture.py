import os
import time

import cv2

import config


class Capture:
    capture_num = 0
    mac_keys = {
        63232: "up",
        63233: "down",
        63234: "left",
        63235: "right",
        27: "esc",
        13: "enter"
    }

    linux_keys = {
        65362: "up",
        65364: "down",
        65361: "left",
        65363: "right",
        'esc': 'esc',
        10: "enter"
    }

    windows_keys = {

    }

    def __init__(self, window_name, width, height, enable_draw=True):
        self.width = width
        self.height = height
        self.enable_draw = enable_draw
        self.window_name = window_name

        self.frame = None

        self.capture = None
        self.video = None

        self.recorder_output_dir = None
        self.recorder_width = None
        self.recorder_height = None

        self.is_running = True

        self.platform = config.get_platform()
        if self.platform == "mac":
            self.key_codes = self.mac_keys
        elif self.platform == "linux":
            self.key_codes = self.linux_keys
        elif self.platform == "win":
            self.key_codes = self.windows_keys
        else:
            raise EnvironmentError('Unsupported platform')

        Capture.capture_num += 1

    def key_pressed(self, delay=1):
        """
        Using OpenCV's waitKey, get the user's pressed key.

        :param delay: An optional delay (milliseconds) to wait for user input.
                Default is 1.
        :return: The pressed key as a single character string OR if the key
                number matches Capture.mac_keys, it will return the
                corresponding text. Type help(Capture.mac_keys) for details.
        """
        key = cv2.waitKey(delay)
        # if key != -1 and self.platform == "linux":
        #     key -= 0x100000
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
            print("Frame saved as " + str(image_name))

        if frame is None:
            frame = self.frame

        if directory is None:
            directory = config.get_dir(":images")
        print("in directory:\n" + directory)

        if not os.path.isdir(directory):
            os.makedirs(directory)
        if directory[-1] != "/":
            directory += "/"

        cv2.imwrite(directory + image_name, frame)

    def get_frame(self):
        pass

    def show_frame(self, frame=None):
        """
        Display the frame in the Capture's window using cv2.imshow

        :param frame: A numpy array containing the image to be displayed
                (shape = (height, width, 3))
        :return: None
        """
        if frame is not None:
            cv2.imshow(self.window_name, frame)
        else:
            cv2.imshow(self.window_name, self.frame)

    def start_recording(self, fps=30, video_name=None, add_timestamp=True,
                        format=None, output_dir=None, width=None, height=None):
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

        if format is None:
            if self.platform == 'mac':
                video_format = 'mov'
                codec = 'mp4v'
            elif self.platform == 'linux':
                video_format = 'avi'
                codec = 'MJPG'
            else:
                raise EnvironmentError('Unsupported platform. Untested.')
        else:
            if format.lower() == 'mov':
                video_format = 'mov'
                codec = 'mp4v'
            else:
                video_format = 'avi'
                codec = 'MJPG'

        if video_name is None:
            video_name = ""
        elif video_name is not None and add_timestamp:
            video_name += " "

        if add_timestamp:
            video_name += time.strftime("%c").replace(":", ";") + "-" + str(
                Capture.capture_num)

        video_name += "." + video_format

        if output_dir is None:
            output_dir = config.get_dir(":videos")
        else:
            output_dir += "/"

        if not os.path.isdir(output_dir):
            os.makedirs(output_dir)

        output_dir += video_name

        fourcc = cv2.VideoWriter_fourcc(*codec)
        self.video = cv2.VideoWriter()

        # ubuntu doesn't like putting videos in other directories...
        if self.platform == 'linux':
            output_dir = video_name

        if width is None:
            self.recorder_width = self.width
        else:
            self.recorder_width = width

        if height is None:
            self.recorder_height = self.height
        else:
            self.recorder_height = height

        self.video.open(output_dir, fourcc, fps,
                        (self.recorder_width, self.recorder_height), True)

        self.recorder_output_dir = output_dir
        print("Initialized video named '%s'." % video_name)

    def record_frame(self, frame):
        """
            Write the frame to the Capture's initialized video capture.
            Type help(Capture.startVideo) for details.

            :param frame: A numpy array containing the frame to write
                    (shape = (height, width, 3))
            :return: None
            """
        if frame.shape[0:2] != (self.recorder_height, self.recorder_width):
            frame = cv2.resize(frame,
                               (self.recorder_height, self.recorder_width))
        if len(frame.shape) == 2:
            self.video.write(cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR))
        else:
            self.video.write(frame)

    def stop_video(self):
        """
            Close the initialized video capture.
            Type help(Capture.startVideo) for details.

            :return: None
            """
        if self.video is not None:
            self.video.release()
            print("Video written to:\n" + self.recorder_output_dir)

    def stop(self):
        self.is_running = False
        if self.capture is not None:
            self.capture.release()
        self.stop_video()
        if self.enable_draw:
            cv2.destroyWindow(self.window_name)
