import cv2

from atlasbuggy import project
from atlasbuggy.vision.capture import Capture


class Video(Capture):
    """
    A wrapper class for opencv's video capture functionality.
    Only accepts the avi, mov, and mp4 video formats
    """
    def __init__(self, video_name, directory=None, enable_draw=True,
                 start_frame=0, width=None, height=None, frame_skip=0,
                 loop_video=False):
        """
        :param video_name: the file name of the video
        :param directory: directory of the video. Uses the default directory
            (named videos) if None provided
        :param enable_draw: whether the opencv window should be shown
            (boosts frames per second)
        :param start_frame: frame number to start the video at
        :param width: set a width for the video
        :param height: set a height for the video
        :param frame_skip: number of frames to skip every iteration
        :param loop_video: if True the stream will jump back to the beginning
            of the video, else it will end the stream
        """
        super(Video, self).__init__(width, height, video_name, enable_draw)

        self.resize_width = width
        self.resize_height = height

        video_name, capture, length_msec, num_frames, self.slider_ticks, \
            self.track_bar_name = self.load_video(video_name, directory)

        self.width, self.height, self.resize_width, self.resize_height, \
            self.resize_frame = self.init_dimensions(
                self.resize_width, self.resize_height, capture)

        # other video properties
        self.frame_skip = frame_skip
        self.loop_video = loop_video

        self.video_name = video_name

        self.capture = capture

        self.video_len = num_frames

        self.slider_has_moved = False

        if start_frame > 0:
            self.set_frame(start_frame)

    def init_dimensions(self, resize_width, resize_height, capture):
        width, height = int(capture.get(
            cv2.CAP_PROP_FRAME_WIDTH)), int(capture.get(
            cv2.CAP_PROP_FRAME_HEIGHT))

        # only resize the frame if the width and height of the video don't
        # match the given width and height
        if (resize_width is not None or resize_height is not None and
                (width, height) != (
                    resize_width, resize_height)):
            resize_frame = True
        else:
            resize_frame = False

        if resize_height is None and width is not None:
            resize_height = int(width * height / width)
        if resize_width is None and height is not None:
            resize_width = int(height * width / height)

        return width, height, resize_width, resize_height, resize_frame

    def show_frame(self, frame=None):
        """
        Display the frame in the Capture's window using cv2.imshow
        If no frame is provided, the previous frame is used.
        """
        if frame is not None:
            cv2.imshow(self.video_name, frame)
        else:
            cv2.imshow(self.video_name, self.frame)

    def load_video(self, video_name, directory):
        """Load a video file from a directory into an opencv capture object"""
        directory = project.parse_dir(directory, ":videos")
        video_name = project.get_file_name(video_name, directory,
                                           ['avi', 'mov', 'mp4',
                                            'AVI', 'MOV', 'MP4'])

        print("loading video into window named '" + str(
            video_name) + "'...")

        capture = cv2.VideoCapture(directory + video_name)

        cv2.namedWindow(video_name)

        # set the properties of the video
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

        # initialize the track bar and the number of ticks it has
        slider_ticks = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH) / 3)

        if slider_ticks > num_frames:
            slider_ticks = num_frames
        track_bar_name = "frame:"
        cv2.createTrackbar(track_bar_name, video_name, 0, slider_ticks,
                           self.on_slider)

        print("video loaded!")
        return video_name, capture, length_msec, num_frames, slider_ticks, track_bar_name

    def get_frame(self, advance_frame=True):
        """Get a new frame from the video stream and return it"""
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
        """Get the current frame number of the video"""
        return int(self.capture.get(cv2.CAP_PROP_POS_FRAMES))

    def on_slider(self, slider_index):
        """When the slider moves, change the video's position"""
        self.slider_has_moved = True
        slider_pos = int(slider_index * self.video_len / self.slider_ticks)
        if abs(slider_pos - self.current_pos()) > 1:
            self.set_frame(slider_pos)
            self.show_frame(self.get_frame())
            self.frame_num = self.current_pos()
            self.slider_num = slider_index

    def slider_moved(self):
        """For external use. Check whether the slider moved involuntarily"""
        if self.slider_has_moved:
            self.slider_has_moved = False
            return True
        else:
            return False

    def set_frame(self, position):
        """Jump the stream to a frame number"""
        if position >= self.video_len:
            position = self.video_len
        if position >= 0:
            self.capture.set(cv2.CAP_PROP_POS_FRAMES, int(position))

    def increment_frame(self):
        """Jump the stream forward one frame"""
        self.get_frame()
        self.slider_has_moved = True

    def decrement_frame(self):
        """Jump the stream backward one frame"""

        # it doesn't listen to me if I don't subtract 3...
        self.set_frame(self.current_pos() - 3)
        self.get_frame()
        self.slider_has_moved = True
