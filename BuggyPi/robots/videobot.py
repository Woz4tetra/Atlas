import sys

sys.path.insert(0, "../")

from robots.robot import Robot
from vision.video import Video


class VideoBot(Robot):
    def __init__(self, file_name, directory, properties):
        super(VideoBot, self).__init__(properties)

        self.enable_camera = True
        start_frame = self.get_property(
            properties, 'start_frame', 0)
        width = self.get_property(
            properties, 'cam_width', None)
        height = self.get_property(
            properties, 'cam_height', None)
        frame_skip = self.get_property(
            properties, 'frame_skip', 0)
        loop_video = self.get_property(
            properties, 'loop_video', False)

        self.capture = Video(file_name, directory, directory, start_frame,
                             width, height, frame_skip, loop_video)

        self.update_fn = self.get_property(
            properties, 'update_camera_fn', lambda x: x)
        self.show_original = self.get_property(
            properties, 'show_original', False)

        self.capture.get_frame(False)

    def update(self):
        if not self.capture.paused:
            if self.capture.get_frame() is None:
                return False
        return self.update_fn(self)
