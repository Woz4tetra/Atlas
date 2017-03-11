import os
import sys

sys.path.insert(0, "..")
os.chdir("..")

from atlasbuggy.interface import RobotSimulator
from atlasbuggy.robot import Robot
from atlasbuggy.capture import Capture


class CaptureTester(Robot):
    def __init__(self):
        super(CaptureTester, self).__init__()
        self.capture = Capture("some capture", enable_recording=False)

    def play(self, video_name, video_dir):
        self.capture.play_video(video_name, video_dir)

    def start_cam(self, video_name, video_dir):
        return self.capture.start_camera(video_name + " " + self.capture.name + ".avi", video_dir)

    def loop(self):
        self.capture.get_frame(self.dt())
        key = self.capture.show_frame()
        if key == 'q':
            return "done"

    def close(self, reason):
        self.capture.close()


file_name, directory = "15;13", "data_days/2017_Mar_05"

video_tester = CaptureTester()
simulator = RobotSimulator(file_name, directory, video_tester)

status = video_tester.start_cam(simulator.file_name_no_ext, simulator.input_dir)
if status is None:
    simulator.run()
