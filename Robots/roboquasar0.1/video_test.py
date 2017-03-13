from atlasbuggy.interface import RobotSimulator
from atlasbuggy.interface import RobotRunner
from atlasbuggy.robot import Robot
from atlasbuggy.vision.camera import Camera
import time

from multiprocessing import Process, Queue, Lock, Event


class Pipeline(Process):
    def __init__(self):
        self.camera = Camera("pipeline")
        self.camera.launch_camera(
            "pipeline", ".", False, capture_number=0
        )

        self.pipeline_queue = Queue()
        self.pipeline_lock = Lock()
        self.exit_event = Event()
        self.exit_lock = Lock()

        super(Pipeline, self).__init__(target=self.update)

    def update(self):
        while True:
            print("something 1")
            with self.exit_lock:
                print("something 2")
                if self.exit_event.is_set():
                    break
                print("something 3")

            print("something 4")
            with self.pipeline_lock:
                print("something 5")
                self.camera.get_frame()
                print("something 6")
                self.pipeline_queue.put(self.camera.frame)
                print("something 7")

    def show_frame(self):
        with self.pipeline_lock:
            while not self.pipeline_queue.empty():
                self.camera.show_frame(self.pipeline_queue.get())

    def close(self):
        with self.exit_lock:
            self.exit_event.set()


class CaptureTester(Robot):
    def __init__(self, enable_recording):
        super(CaptureTester, self).__init__()
        self.logitech = Camera("logitech", width=300, height=200)
        self.pipeline = Pipeline(self.logitech)
        self.ps3eye = Camera("ps3eye")

        self.start_time = time.time()

        super(CaptureTester, self).__init__()

    def start(self):
        logitech_name = "%s%s.avi" % (self.get_path_info("file name no extension"), self.logitech.name)
        ps3eye_name = "%s%s.avi" % (self.get_path_info("file name no extension"), self.ps3eye.name)
        directory = self.get_path_info("input dir")

        if self.is_live:
            status = self.logitech.launch_camera(
                logitech_name, directory, self.logger.is_open(),
            )
            if status is not None:
                return status

            status = self.ps3eye.launch_camera(
                ps3eye_name, directory, self.logger.is_open(),
            )
            if status is not None:
                return status
        else:
            self.logitech.launch_video(logitech_name, directory)
            self.ps3eye.launch_video(ps3eye_name, directory)

        self.pipeline.start()
        self.start_time = time.time()

    def loop(self):
        self.pipeline.show_frame()
        if self.logitech.get_frame(self.dt()) is None:
            return "exit"

        if self.ps3eye.get_frame(self.dt()) is None:
            return "exit"

        key = self.logitech.show_frame()
        if key == 'q' or key == -2:
            return "done"

        key = self.ps3eye.show_frame()
        if key == 'q':
            return "done"

    def close(self, reason):
        self.logitech.close()
        self.ps3eye.close()


class PipelineTest:
    def __init__(self):
        self.pipeline = Pipeline()

        self.pipeline.start()

    def show(self):
        self.pipeline.show_frame()


# def run(live):
#     video_tester = CaptureTester(False)
#
#     if live:
#         runner = RobotRunner(video_tester, log_data=False)
#         runner.run()
#     else:
#         simulator = RobotSimulator("18;54", "2017_Mar_12", video_tester)
#         simulator.run()
# run(True)

def pipeline():
    test = PipelineTest()
    while True:
        test.show()


pipeline()
