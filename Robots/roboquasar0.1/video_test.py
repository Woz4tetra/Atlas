
from atlasbuggy.interface import RobotSimulator
from atlasbuggy.interface import RobotRunner
from atlasbuggy.robot import Robot
from atlasbuggy.vision.capture import Capture


class CaptureTester(Robot):
    def __init__(self, enable_recording):
        super(CaptureTester, self).__init__()
        self.logitech = Capture("logitech", enable_recording=enable_recording)
        self.ps3eye = Capture("ps3eye", enable_recording=enable_recording)

        super(CaptureTester, self).__init__()

    def play(self, video_name, video_dir):
        self.logitech.play_video(video_name + self.logitech.name, video_dir)
        self.ps3eye.play_video(video_name + self.logitech.ps3eye, video_dir)

    def start_cam(self, video_name, video_dir):
        status = self.logitech.start_camera("%s%s.avi" % (video_name, self.logitech.name), video_dir)
        if status is not None:
            return status

        status = self.ps3eye.start_camera("%s%s.avi" % (video_name, self.ps3eye.name), video_dir)
        if status is not None:
            return status

    def loop(self):
        self.logitech.get_frame(self.dt())
        self.ps3eye.get_frame(self.dt())

        key = self.logitech.show_frame()
        if key == 'q':
            return "done"

        key = self.ps3eye.show_frame()
        if key == 'q':
            return "done"

    def close(self, reason):
        self.logitech.close()
        self.ps3eye.close()


def run(live=True):
    video_tester = CaptureTester(False)

    if live:
        runner = RobotRunner(video_tester, log_data=False)
        file_name, directory = runner.get_path()
        status = video_tester.start_cam(file_name, directory)
        if status is None:
            runner.run()
    else:
        file_name, directory = "15;13", "data_days/2017_Mar_05"

        simulator = RobotSimulator(file_name, directory, video_tester)
        file_name, directory = simulator.get_path()
        status = video_tester.play(file_name, directory)
        if status is None:
            simulator.run()

run()
