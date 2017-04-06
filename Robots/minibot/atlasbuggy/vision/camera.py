import os
from atlasbuggy.files.videofile import *
from atlasbuggy import get_platform


class Camera:
    captures = {}
    used_captures = set()
    min_cap_num = 0
    max_cap_num = None

    def __init__(self, name, width=None, height=None, show=True, skip_count=0, enabled=True):
        self.file_name = ""
        self.directory = ""
        self.full_path = ""

        self.is_live = None

        self.name = name
        self.capture = None
        self.frame = None
        self.show = show
        self.record = False
        self.enabled = enabled

        self.fps = 0.0
        self.fps_sum = 0.0
        self.num_frames = 0
        self.length_sec = 0.0
        self.length_msec = 0.0
        self.start_time = 0.0
        self.current_time = 0.0
        self.prev_t = None

        self.width = width
        self.height = height
        self.resize_frame = False

        self.sync_up_error = None
        self.video_writer = None
        self.is_recording = False

        self.start_frame = 0
        self.loop_video = False
        self.frame_skip_count = skip_count

        platform = get_platform()
        if platform == "linux":
            self.key_codes = {
                65362: "up",
                65364: "down",
                65361: "left",
                65363: "right",
            }
        elif platform == "mac":
            self.key_codes = {
                63232: "up",
                63233: "down",
                63234: "left",
                63235: "right",
            }
        else:
            self.key_codes = {}

    def launch_camera(self, file_name, directory, record, capture_number=None):
        self.is_live = True

        video_name, video_dir = AtlasReadFile.format_path_as_time(
            file_name, directory,
            default_log_dir_name, default_log_file_name
        )

        write_file = AtlasWriteFile(video_name, video_dir, False, ("avi", "mp4", "mov"), "videos")
        self.file_name = write_file.file_name
        self.directory = write_file.directory
        self.full_path = write_file.full_path

        self.get_frame = self.get_frame_camera

        self.record = record

        if not self.enabled:
            return None

        if capture_number is None:
            capture, height, width = self.launch_selector()
            if type(capture) == str:
                raise FileNotFoundError(capture)
            if capture is None:
                return "exit"
            self.capture = capture
        else:
            self.capture = self.load_capture(capture_number)
            success, frame = self.capture.read()
            if not success:
                raise FileNotFoundError("Camera %s failed to load!" % capture_number)
            height, width = frame.shape[0:2]

        if self.height is not None:
            if height != self.height:
                self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                self.resize_frame = True
        else:
            self.height = height

        if self.width is not None:
            if width != self.width:
                self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                self.resize_frame = True
        else:
            self.width = width

        self.start_time = time.time()
        if self.show:
            cv2.namedWindow(self.name)

    def launch_video(self, file_name, directory, loop=False, sync_up_error=0.001, start_frame=0):
        self.is_live = False

        self.loop_video = loop
        self.sync_up_error = sync_up_error
        self.start_frame = start_frame

        self.get_frame = self.get_frame_video

        if not self.enabled:
            return None

        read_file = AtlasReadFile(file_name, directory, False, ("avi", "mp4", "mov"), "videos")

        self.file_name = read_file.file_name
        self.directory = read_file.directory
        self.full_path = read_file.full_path


        if not self.enabled:
            return None

        if not os.path.isfile(self.full_path):
            print("Warning: video '%s' not found. Disabling camera (%s)" % (self.full_path, self.name))
            self.enabled = False
            return None

        print("Using video:", self.full_path)
        self.capture = cv2.VideoCapture(self.full_path)
        self.fps = self.capture.get(cv2.CAP_PROP_FPS)
        self.num_frames = int(self.capture.get(cv2.CAP_PROP_FRAME_COUNT))
        if self.num_frames <= 0:
            raise FileNotFoundError("Video failed to load!")
        self.length_sec = self.num_frames / self.fps
        self.length_msec = int(self.length_sec * 1000)

        width = int(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))

        if self.height is not None:
            if height != self.height:
                self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                self.resize_frame = True
        else:
            self.height = height

        if self.width is not None:
            if width != self.width:
                self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                self.resize_frame = True
        else:
            self.width = width

        if start_frame > 0:
            self.set_frame_pos(start_frame)
            print("Starting at frame %s of %s" % (self.current_pos(), self.num_frames))

    def launch_selector(self):
        selector_window_name = "Select camera for: " + self.name
        selected_capture = None
        current_capture = None
        current_num = 0
        width = None
        height = None

        while current_num in Camera.used_captures:
            current_num += 1

            if Camera.max_cap_num is not None and current_num > Camera.max_cap_num:
                return "No cameras left!", height, width

            try:
                current_capture = self.load_capture(current_num)
                success, frame = current_capture.read()
                if not success:
                    raise cv2.error
            except cv2.error:
                Camera.max_cap_num = current_num - 1
                current_capture.release()
                return "No cameras left!", height, width

        current_capture = self.load_capture(current_num)

        while selected_capture is None:
            key = self.key_pressed()

            if key == "left":
                current_num -= 1
                if current_num < Camera.min_cap_num:
                    current_num = Camera.min_cap_num
                    print("Camera failed to load! Camera number lower limit:", current_num)
                    continue
                while current_num in Camera.used_captures:
                    current_num -= 1

                current_capture = self.load_capture(current_num)

            elif key == "right":
                current_num += 1
                if Camera.max_cap_num is not None and current_num > Camera.max_cap_num:
                    print("Camera failed to load! Camera number upper limit:", current_num)
                    current_num = Camera.max_cap_num
                    continue

                while current_num in Camera.used_captures:
                    current_num += 1

                try:
                    current_capture = self.load_capture(current_num)
                    success, frame = current_capture.read()
                    cv2.imshow(selector_window_name, frame)
                except cv2.error:
                    print("Camera failed to load! Camera number upper limit:", current_num)
                    if current_num in Camera.captures:
                        current_capture.release()
                        del Camera.captures[current_num]
                    current_num -= 1
                    Camera.max_cap_num = current_num
                    current_capture = self.load_capture(current_num)

            elif key == "\n" or key == "\r":
                selected_capture = current_capture
                Camera.used_captures.add(current_num)
                print("Using capture #%s for %s" % (current_num, self.name))

            elif key == 'q':
                selected_capture = None
                break

            success, frame = current_capture.read()
            cv2.imshow(selector_window_name, frame)
            height, width = frame.shape[0:2]
        cv2.destroyWindow(selector_window_name)
        return selected_capture, height, width

    def current_pos(self):
        return self.num_frames if self.is_live else int(self.capture.get(cv2.CAP_PROP_POS_FRAMES))

    def set_frame_pos(self, position):
        if not self.is_live:
            if position >= self.num_frames:
                position = self.num_frames
            if position >= 0:
                self.capture.set(cv2.CAP_PROP_POS_FRAMES, int(position))

    def get_frame(self, timestamp=None):
        raise Exception("launch_camera or launch_video not called!")

    def get_frame_camera(self, timestamp=None):
        if not self.enabled:
            return 0
        success, self.frame = self.capture.read()
        if not success:
            return None
        if self.resize_frame and self.frame.shape[0:2] != (self.height, self.width):
            self.frame = cv2.resize(self.frame, (self.width, self.height))

        self.poll_for_fps()
        self.num_frames += 1

        if self.record:
            self.record_frame(self.frame)

        return self.frame

    def get_frame_video(self, timestamp=None):
        # t0 = time.time()
        if not self.enabled:
            return 0
        if timestamp is not None:
            self.current_time = self.current_pos() * self.length_sec / self.num_frames
            if abs(timestamp - self.current_time) > self.sync_up_error:
                if timestamp < self.current_time:
                    return self.frame
                else:
                    goal_frame = int(timestamp * self.num_frames / self.length_sec)
                    self.set_frame_pos(goal_frame)
                    return self.get_frame()

        if self.frame_skip_count > 0:
            self.set_frame_pos(self.current_pos() + self.frame_skip_count)

        # t1 = time.time()
        success, self.frame = self.capture.read()
        # t2 = time.time()
        # print(t1 - t0)
        # print(t2 - t1)
        # print()

        if not success or self.frame is None:
            if self.loop_video:
                self.set_frame_pos(0)
                while success is False or self.frame is None:
                    success, self.frame = self.capture.read()
            else:
                self.close()
                return None
        if self.resize_frame:
            self.frame = cv2.resize(self.frame,
                                    (self.width, self.height),
                                    interpolation=cv2.INTER_NEAREST)

        return self.frame

    def show_frame(self, frame=None):
        """
        Display the frame in the Capture's window using cv2.imshow
        :param frame: A numpy array containing the image to be displayed
                (shape = (height, width, 3))
        :return: None
        """
        if self.show:
            if frame is None:
                frame = self.frame
                if frame is None:
                    return -2

            cv2.imshow(self.name, frame)

    def load_capture(self, arg):
        if arg not in Camera.captures:
            print("Loading capture '%s'..." % arg, end="")
            Camera.captures[arg] = cv2.VideoCapture(arg)
            print("done")
        return Camera.captures[arg]

    def poll_for_fps(self):
        if self.prev_t is None:
            self.prev_t = time.time()
            return 0.0

        self.length_sec = time.time() - self.start_time
        self.length_msec = self.length_sec * 1000
        self.fps = 1 / (time.time() - self.prev_t)
        self.fps_sum += self.fps

        self.prev_t = time.time()

    def record_frame(self, frame):
        if self.num_frames == 25:
            self.init_recording()
            print("Writing video to: '%s'. FPS: %0.2f" % (self.full_path, self.fps_sum / self.num_frames))
        elif self.num_frames > 25:
            if frame.shape[0:2] != (self.height, self.width):
                frame = cv2.resize(frame, (self.height, self.width))
            if len(frame.shape) == 2:
                self.video_writer.write(cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR))
            else:
                self.video_writer.write(frame)

    def init_recording(self):
        if not self.is_recording:
            if self.file_name.endswith('avi'):
                codec = 'MJPG'
            elif self.file_name.endswith('mp4'):
                if get_platform() == 'mac':
                    codec = 'mp4v'
                else:
                    # TODO: Figure out mp4 recording in ubuntu
                    # codec = 'X264'
                    codec = 'MJPG'
                    self.file_name = self.file_name[:-3] + "avi"
                    self.full_path = self.full_path[:-3] + "avi"
            else:
                raise ValueError("Invalid file format")
            fourcc = cv2.VideoWriter_fourcc(*codec)
            self.video_writer = cv2.VideoWriter()
            self.video_writer.open(self.full_path, fourcc, self.fps_sum / self.num_frames, (self.width, self.height), True)
            self.is_recording = True

    def add_key(self, key_num, value):
        self.key_codes[key_num] = value

    def key_pressed(self, delay=1):
        if not self.enabled:
            return -1
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

    def close(self, only_unused=False):
        for cap_name, capture in Camera.captures.items():
            if only_unused:
                if cap_name not in Camera.used_captures:
                    capture.release()
            else:
                capture.release()

        if self.is_recording:
            self.video_writer.release()
            self.is_recording = False

        cv2.destroyWindow(self.name)
