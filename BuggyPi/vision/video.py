import os
import cv2

import project
from vision.capture import Capture

class Video(Capture):
    def __init__(self, video_name, directory=None, enable_draw=True,
                 start_frame=0, width=None, height=None, frame_skip=0,
                 loop_video=False):
        video_name, capture, length_msec, num_frames, self.slider_ticks, self.track_bar_name = \
            self.load_video(video_name, directory)

        super(Video, self).__init__(width, height, video_name, enable_draw)
        
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

        assert video_name.endswith('avi')
        self.video_name = video_name

        self.capture = capture
        
        self.video_len = num_frames
        
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
        if directory is None:
            directory = project.get_dir(":videos")
        elif os.path.isdir(project.get_dir(":videos") + directory):
            if directory[-1] != "/":
                directory += "/"
            capture = cv2.VideoCapture(
                project.get_dir(":videos") + directory + video_name)
        else:
            raise NotADirectoryError("Invalid directory: " + str(directory))

        if type(video_name) == int:
            files = sorted(os.listdir(directory))
            video_files = []
            for file in files:
                if len(file) >= 4 and file[-4:] == '.avi':
                    video_files.append(file)
            # video_name is the index in the list of files in the directory
            video_name = video_files[video_name]
            print("Using video named '%s'" % video_name)
        
        print("loading video into window named '" + str(
            video_name) + "'...")
        
        capture = cv2.VideoCapture(directory + video_name)
        
##        if self.enable_draw:
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

        slider_ticks = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH) // 3)
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
##        if self.frame.shape[0] == 0 or self.frame.shape[1] == 0:
##            success = False
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
##        if self.platform != "linux":
        if self.current_pos() != self.frame_num:
            self.frame_num = self.current_pos()
            self.slider_num = int(self.frame_num * self.video_len / self.slider_ticks)
            cv2.setTrackbarPos(self.track_bar_name, self.video_name,
                               self.slider_num)

        return self.frame
    
    def current_pos(self):
        return int(self.capture.get(cv2.CAP_PROP_POS_FRAMES))

    def on_slider(self, slider_index):
        slider_pos = int(slider_index * self.video_len / self.slider_ticks)
        if abs(slider_pos - self.current_pos()) > 1:
            self.set_frame(slider_pos)
            self.show_frame(self.get_frame())
            self.frame_num = self.current_pos()
            self.slider_num = slider_index

    def set_frame(self, position):
        if position >= self.video_len:
            position = self.video_len
        if position >= 0:
            self.capture.set(cv2.CAP_PROP_POS_FRAMES, int(position))

    def increment_frame(self):
#        self.set_frame(self.current_pos() + 1)
        self.get_frame()

    def decrement_frame(self):
        self.set_frame(self.current_pos() - 2)
        self.get_frame(False)
