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
        self.capture1 = Capture("some capture 1", enable_recording=False)
        self.capture2 = Capture("some capture 2", enable_recording=False)

    def play(self, video_name, video_dir):
        self.capture2.play_video(video_name, video_dir)

    def start_cam(self, video_name, video_dir):
        status = self.capture1.start_camera(video_name + " " + self.capture1.name + ".avi", video_dir, 1)
        if status is not None:
            return status

        status = self.capture2.start_camera(video_name + " " + self.capture2.name + ".avi", video_dir, 2)
        if status is not None:
            return status

    def loop(self):
        self.capture1.get_frame(self.dt())
        self.capture2.get_frame(self.dt())

        key = self.capture1.show_frame()
        if key == 'q':
            return "done"

        key = self.capture2.show_frame()
        if key == 'q':
            return "done"

    def close(self, reason):
        self.capture2.close()


file_name, directory = "15;13", "data_days/2017_Mar_05"

video_tester = CaptureTester()
simulator = RobotSimulator(file_name, directory, video_tester)

status = video_tester.start_cam(simulator.file_name_no_ext, simulator.input_dir)
if status is None:
    simulator.run()
