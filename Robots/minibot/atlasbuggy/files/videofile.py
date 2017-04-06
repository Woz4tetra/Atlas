import cv2
import time
from atlasbuggy.files.atlasbuggyfile import AtlasWriteFile, AtlasReadFile
from atlasbuggy.files.logfile import default_log_dir_name, default_log_file_name


class VideoPlayer:
    def __init__(self, video_name, video_dir, window_name, capture, width=None, height=None, frame_skip=0,
                 loop_video=False, start_frame=0, slider_callback=None):
        video_name, video_dir = AtlasReadFile.format_path_as_time(video_name, video_dir, default_log_dir_name,
                                                                  default_log_file_name)

        self.read_file = AtlasReadFile(video_name, video_dir, False, ("avi", "mov"), "videos")

        self.window_name = window_name

        self.frame = None
        self.current_frame_num = 0
        self.current_time = 0.0

        self.capture = capture
        self.cv_capture = cv2.VideoCapture(self.full_path)
        cv2.namedWindow(self.window_name)

        self.fps = self.cv_capture.get(cv2.CAP_PROP_FPS)
        self.num_frames = int(self.cv_capture.get(cv2.CAP_PROP_FRAME_COUNT))
        if self.num_frames <= 0:
            raise FileNotFoundError("Video failed to load!")

        self.length_sec = self.num_frames / self.fps
        self.length_msec = int(self.length_sec * 1000)

        self.slider_pos = 0
        self.slider_ticks = int(self.cv_capture.get(cv2.CAP_PROP_FRAME_WIDTH) // 3)
        if self.slider_ticks > self.num_frames:
            self.slider_ticks = self.num_frames
        self.track_bar_name = "frame:"
        cv2.createTrackbar(self.track_bar_name, self.window_name, 0, self.slider_ticks,
                           self.on_slider)
        self.slider_callback = slider_callback

        self.width = int(self.cv_capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cv_capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.resize_frame = False

        if width is None:
            self.resize_width = self.width
        else:
            self.resize_width = width
            self.resize_frame = True

        if height is None:
            self.resize_height = self.height
        else:
            self.resize_height = height
            self.resize_frame = True

        self.frame_skip = frame_skip
        self.loop_video = loop_video

        if start_frame > 0:
            self.set_frame(start_frame)

        self.sync_up_error = 0.01

    def video_len(self):
        return self.num_frames

    def current_pos(self):
        return int(self.cv_capture.get(cv2.CAP_PROP_POS_FRAMES))

    def on_slider(self, slider_index):
        slider_pos = int(slider_index * self.video_len() / self.slider_ticks)
        if abs(slider_pos - self.current_pos()) > 1:
            self.set_frame(slider_pos)
            self.capture.show_frame(self.get_frame())
            self.current_frame_num = self.current_pos()
            self.slider_pos = slider_index
            if self.slider_callback is not None:
                self.slider_callback()

    def set_frame(self, position):
        if position >= self.video_len():
            position = self.video_len()
        if position >= 0:
            self.cv_capture.set(cv2.CAP_PROP_POS_FRAMES, int(position))

    def get_frame(self, current_time=None, advance_frame=True):
        if current_time is not None:
            self.current_time = self.current_pos() * self.length_sec / self.num_frames
            if abs(current_time - self.current_time) > self.sync_up_error:
                goal_frame = int(current_time * self.num_frames / self.length_sec)
                self.set_frame(goal_frame)
                return self.get_frame()

        if self.frame_skip > 0:
            self.set_frame(self.current_pos() + self.frame_skip)

        success, self.frame = self.cv_capture.read()
        if not advance_frame:
            self.set_frame(self.current_pos() - 1)

        if not success or self.frame is None:
            if self.loop_video:
                self.set_frame(0)
                while success is False or self.frame is None:
                    success, self.frame = self.cv_capture.read()
            else:
                self.close()
                return None
        if self.resize_frame:
            self.frame = cv2.resize(self.frame,
                                    (self.resize_width, self.resize_height),
                                    interpolation=cv2.INTER_NEAREST)
        if self.current_pos() != self.current_frame_num:
            self.current_frame_num = self.current_pos()
            self.slider_pos = int(self.current_frame_num * self.slider_ticks / self.video_len())
            cv2.setTrackbarPos(self.track_bar_name, self.window_name, self.slider_pos)

        return self.frame

    def close(self):
        cv2.destroyWindow(self.window_name)


class VideoRecorder(AtlasWriteFile):
    def __init__(self, video_name, video_dir, width, height, enable_recording, capture, cam_number, cv_capture):
        super(VideoRecorder, self).__init__(video_name, video_dir, False, "avi", "videos")
        if cv_capture is not None:
            self.cv_capture = cv_capture
        elif cam_number is not None:
            self.cv_capture = cv2.VideoCapture(cam_number)
        else:
            raise ValueError("Capture number or capture instance not supplied!")

        print("Sampling for FPS...", end="")
        time0 = time.time()
        samples = 15
        for frame_num in range(samples):
            success, self.frame = self.cv_capture.read()
            if not success:
                raise FileNotFoundError("Failed to retrieve from camera")
            capture.show_frame(self.frame)
        fps = samples / (time.time() - time0)
        print("done: ", fps)

        self.enable_recording = enable_recording

        self.width = width
        self.height = height

        if width is not None:
            self.recorder_width = width
            self.width = width
        else:
            self.recorder_width = self.frame.shape[1]
            self.width = self.frame.shape[1]

        if height is not None:
            self.recorder_height = height
            self.height = height
        else:
            self.recorder_height = self.frame.shape[0]
            self.height = self.frame.shape[0]

        self.resize_frame = self.frame.shape[0:2] != (self.height, self.width)

        if self.enable_recording:
            codec = 'MJPG'
            fourcc = cv2.VideoWriter_fourcc(*codec)
            self.video = cv2.VideoWriter()
            self.video.open(self.full_path, fourcc, fps, (self.recorder_width, self.recorder_height), True)

            self._is_open = True

            print("Writing video to:", self.full_path)
        else:
            self.video = None

    def write(self, frame):
        if frame.shape[0:2] != (self.recorder_height, self.recorder_width):
            frame = cv2.resize(frame, (self.recorder_height, self.recorder_width))
        if len(frame.shape) == 2:
            self.video.write(cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR))
        else:
            self.video.write(frame)

    def get_frame(self):
        success, self.frame = self.cv_capture.read()
        if self.resize_frame and self.frame.shape[0:2] != (self.height, self.width):
            self.frame = cv2.resize(self.frame, (self.width, self.height))

        if self.enable_recording:
            self.write(self.frame)
        return self.frame

    def close(self):
        if self._is_open:
            self.video.release()
            self._is_open = False
            print("Wrote video to:", self.full_path)
