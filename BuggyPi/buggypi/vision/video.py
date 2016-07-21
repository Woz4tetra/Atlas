import cv2

from buggypi import project
from buggypi.vision.capture import Capture


class Video(Capture):
    def __init__(self, video_name, directory=None, enable_draw=True,
                 start_frame=0, width=None, height=None, frame_skip=0,
                 loop_video=False):
        super(Video, self).__init__(width, height, video_name, enable_draw)

        self.resize_width = width
        self.resize_height = height

        if (self.resize_width is not None or self.resize_height is not None and
                (self.width, self.height) != (
                    self.resize_width, self.resize_height)):
            self.resize_frame = True
        else:
            self.resize_frame = False

        if self.resize_height is None and width is not None:
            self.resize_height = int(width * self.height / self.width)
        if self.resize_width is None and height is not None:
            self.resize_width = int(height * self.width / self.height)

        video_name, capture, length_msec, num_frames, self.slider_ticks, self.track_bar_name = \
            self.load_video(video_name, directory)

        self.width, self.height = int(capture.get(
            cv2.CAP_PROP_FRAME_WIDTH)), int(capture.get(
            cv2.CAP_PROP_FRAME_HEIGHT))

        self.frame_skip = frame_skip
        self.loop_video = loop_video

        # assert video_name.endswith('avi')
        self.video_name = video_name

        self.capture = capture

        self.video_len = num_frames

        self.slider_has_moved = False

        if start_frame > 0:
            self.set_frame(start_frame)

    def show_frame(self, frame=None):
        """
        Display the frame in the Capture's window using cv2.imshow

        :param frame: A numpy array containing the image to be displayed
                (shape = (height, width, 3))
        :return: None
        """
        if frame is not None:
            cv2.imshow(self.video_name, frame)
        else:
            cv2.imshow(self.video_name, self.frame)

    def load_video(self, video_name, directory):
        directory = project.parse_dir(directory, ":videos")
        video_name = project.get_file_name(video_name, directory,
                                           ['avi', 'mov', 'mp4',
                                            'AVI', 'MOV', 'MP4'])

        print("loading video into window named '" + str(
            video_name) + "'...")

        capture = cv2.VideoCapture(directory + video_name)

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

        if not self.resize_frame:
            slider_ticks = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH) / 3)
        else:
            slider_ticks = int(self.resize_width / 3)

        if slider_ticks > num_frames:
            slider_ticks = num_frames
        track_bar_name = "frame:"
        cv2.createTrackbar(track_bar_name, video_name, 0, slider_ticks,
                           self.on_slider)

        print("video loaded!")
        return video_name, capture, length_msec, num_frames, slider_ticks, track_bar_name

    def get_frame(self, advance_frame=True):
        if self.frame_skip > 0:
            self.set_frame(self.current_pos() + self.frame_skip)

        success, self.frame = self.capture.read()
        if not advance_frame:
            self.set_frame(self.current_pos() - 1)

        if success is False or self.frame is None:
            if self.loop_video:
                self.set_frame(0)
                while success is False or self.frame is None:
                    success, self.frame = self.capture.read()
            else:
                self.stop()
                return None
        if self.resize_frame:
            self.frame = cv2.resize(self.frame,
                                    (self.resize_width, self.resize_height),
                                    interpolation=cv2.INTER_NEAREST)
        if self.current_pos() != self.frame_num:
            self.frame_num = self.current_pos()
            self.slider_num = int(
                self.frame_num * self.slider_ticks / self.video_len)
            cv2.setTrackbarPos(self.track_bar_name, self.video_name,
                               self.slider_num)
        return self.frame

    def current_pos(self):
        return int(self.capture.get(cv2.CAP_PROP_POS_FRAMES))

    def on_slider(self, slider_index):
        self.slider_has_moved = True
        slider_pos = int(slider_index * self.video_len / self.slider_ticks)
        if abs(slider_pos - self.current_pos()) > 1:
            self.set_frame(slider_pos)
            self.show_frame(self.get_frame())
            self.frame_num = self.current_pos()
            self.slider_num = slider_index

    def slider_moved(self):
        if self.slider_has_moved:
            self.slider_has_moved = False
            return True
        else:
            return False

    def set_frame(self, position):
        if position >= self.video_len:
            position = self.video_len
        if position >= 0:
            self.capture.set(cv2.CAP_PROP_POS_FRAMES, int(position))

    def increment_frame(self):
        self.get_frame()
        self.slider_has_moved = True

    def decrement_frame(self):
        self.set_frame(self.current_pos() - 3)
        self.get_frame()
        self.slider_has_moved = True
