
from atlasbuggy.interface import RobotSimulator
from atlasbuggy.interface import RobotRunner
from atlasbuggy.robot import Robot
from atlasbuggy.vision.camera import Camera
import time

class CaptureTester(Robot):
    def __init__(self, enable_recording):
        super(CaptureTester, self).__init__()
        self.logitech = Camera("logitech", width=300, height=200)
        # self.ps3eye = Camera("ps3eye")

        self.start_time = time.time()

        super(CaptureTester, self).__init__()

    def start(self):
        logitech_name = "%s%s.avi" % (self.get_path_info("file name no extension"), self.logitech.name)
        # ps3eye_name = "%s%s.avi" % (self.get_path_info("file name no extension"), self.ps3eye.name)
        directory = self.get_path_info("input dir")

        if self.is_live:
            status = self.logitech.launch_camera(
                logitech_name, directory, True,#self.logger.is_open(),
            )
            if status is not None:
                return status

            # status = self.ps3eye.launch_camera(
            #     ps3eye_name, directory, self.logger.is_open(),
            # )
            # if status is not None:
            #     return status
        else:
            self.logitech.launch_video(logitech_name, directory)
            # self.ps3eye.launch_video(ps3eye_name, directory)

        self.start_time = time.time()

    def loop(self):
        if self.logitech.get_frame(self.dt()) is None:
            return "exit"
        # self.ps3eye.get_frame(self.dt())

        key = self.logitech.show_frame()
        if key == 'q' or key == -2:
            return "done"

        # key = self.ps3eye.show_frame()
        # if key == 'q':
        #     return "done"

    def close(self, reason):
        self.logitech.close()
        # self.ps3eye.close()
        print(time.time() - self.start_time)
        print(self.dt())


def run(live):
    video_tester = CaptureTester(False)

    if live:
        runner = RobotRunner(video_tester, log_data=False)
        runner.run()
    else:
        simulator = RobotSimulator("18;54", "2017_Mar_12", video_tester)
        simulator.run()

run(False)
