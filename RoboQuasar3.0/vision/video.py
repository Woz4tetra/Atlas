import os

import cv2

import config
from vision.capture import Capture


class Video(Capture):
    def __init__(self, video_name, directory=None, enable_draw=True,
                 start_frame=0, width=None, height=None, frame_skip=0,
                 loop_video=False):
        super(Video, self).__init__(video_name, width, height, enable_draw)

        capture, length_msec, num_frames, self.slider_len, self.track_bar_name = \
            self.load_video(video_name, directory)

        self.resize_width = width
        self.resize_height = height

        if (self.resize_width is not None or self.resize_height is not None and
                (self.width, self.height) != (
                    self.resize_width, self.resize_height)):
            self.resize_frame = True
        else:
            self.resize_frame = False

        self.width, self.height = int(capture.get(
            cv2.CAP_PROP_FRAME_WIDTH)), int(capture.get(
            cv2.CAP_PROP_FRAME_HEIGHT))

        if self.resize_height is None and width is not None:
            self.resize_height = int(width * self.height / self.width)
        if self.resize_width is None and height is not None:
            self.resize_width = int(height * self.width / self.height)

        self.frame_skip = frame_skip
        self.loop_video = loop_video

        video_name = video_name.lower()

        self.capture = capture
        if video_name.endswith('avi'):
            self.video_len = num_frames
            self.current_pos = self.current_frame
            self.set_frame = self.set_frame_pos
        elif video_name.endswith('mov') or video_name.endswith('mp4'):
            self.video_len = length_msec
            self.current_pos = self.current_time_msec
            self.set_frame = self.set_frame_msec
        else:
            raise NotImplementedError("Unsupported video format. Supported "
                                      "formats are avi, mov, and mp4.")

        if start_frame > 0:
            self.set_frame(start_frame)

    def load_video(self, video_name, directory):
        print("loading video into window named '" + str(
            video_name) + "'...")
        if directory is None:
            capture = cv2.VideoCapture(config.get_dir(":videos") + video_name)
        elif os.path.isdir(config.get_dir(":videos") + directory):
            if directory[-1] != "/":
                directory += "/"
            capture = cv2.VideoCapture(
                config.get_dir(":videos") + directory + video_name)
        else:
            raise NotADirectoryError("Invalid directory: " + str(directory))

        if self.enable_draw:
            cv2.namedWindow(video_name)

        fps = capture.get(cv2.CAP_PROP_FPS)
        num_frames = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
        if num_frames <= 0:
            raise Exception("Video failed to load! "
                            "Did you misspell the video name?")

        length_sec = num_frames / fps
        length_msec = int(length_sec * 1000)

        print("\tfps:", fps)
        print("\tlength (sec):", length_sec)
        print("\tlength (frames):", num_frames)

        slider_len = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH) // 3)
        track_bar_name = "frame:"
        cv2.createTrackbar(track_bar_name, video_name, 0, slider_len,
                           self.on_slider)

        print("video loaded!")
        return capture, length_msec, num_frames, slider_len, track_bar_name

    def get_frame(self):
        if self.frame_skip > 0:
            self.set_frame(self.current_pos() + self.frame_skip)

        success, self.frame = self.capture.read()

        if success is False or self.frame is None:
            if self.loop_video:
                self.set_frame(0)
                while success is False or self.frame is None:
                    success, self.frame = self.capture.read()
            else:
                self.stop()
                return
        if self.resize_frame:
            self.frame = cv2.resize(self.frame,
                                    (self.resize_width, self.resize_height),
                                    interpolation=cv2.INTER_NEAREST)
        if self.platform != "linux":
            cv2.setTrackbarPos(self.track_bar_name, self.window_name,
                               int(self.current_pos() *
                                   self.slider_len / self.video_len))

        return self.frame

    # --- video position methods --- #
    def current_pos(self):
        return 0

    def current_time_msec(self):
        return int(self.capture.get(cv2.CAP_PROP_POS_MSEC))

    def current_frame(self):
        return int(self.capture.get(cv2.CAP_PROP_POS_FRAMES))

    # --- slider methods --- #
    def on_slider(self, slider_index):
        slider_time = int(slider_index * self.video_len / self.slider_len)
        if abs(slider_time - self.current_pos()) > 10:
            self.set_frame(slider_time)
            self.show_frame(self.get_frame())

    # --- position setter methods --- #
    def set_frame(self, position):
        pass

    def set_frame_pos(self, position):
        if position >= self.video_len:
            position = self.video_len
        if position >= 0:
            self.capture.set(cv2.CAP_PROP_POS_FRAMES, int(position))

    def set_frame_msec(self, time_msec):
        if time_msec >= self.video_len:
            time_msec = self.video_len
        if time_msec >= 0:
            self.capture.set(cv2.CAP_PROP_POS_MSEC, int(time_msec))

    def increment_frame(self):
        self.set_frame(self.current_pos() + 1)

    def decrement_frame(self):
        self.set_frame(self.current_pos() - 2)
