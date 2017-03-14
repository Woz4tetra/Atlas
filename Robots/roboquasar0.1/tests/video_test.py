from atlasbuggy.interface import RobotSimulator
from atlasbuggy.interface import RobotRunner
from atlasbuggy.robot import Robot
from atlasbuggy.vision.camera import Camera
import time
import cv2

from multiprocessing import Process, Queue, Lock, Event


class Pipeline(Process):
    def __init__(self):
        self.frames_queue = Queue()
        self.results_queue = Queue()
        self.pipeline_lock = Lock()
        self.exit_event = Event()
        self.exit_lock = Lock()
        self.capture = cv2.VideoCapture(1)

        super(Pipeline, self).__init__(target=self.update)

    def update(self):
        while True:
            success, frame = self.capture.read()
            with self.exit_lock:
                if self.exit_event.is_set():
                    break

            with self.pipeline_lock:
                self.results_queue.put(self.pipeline(frame))
                print("1", self.results_queue.empty())

    def pipeline(self, frame):
        return cv2.medianBlur(frame, 111)

    def put(self, frame):
        with self.pipeline_lock:
            self.frames_queue.put(frame)

    def close(self):
        with self.exit_lock:
            self.exit_event.set()


class CaptureTester(Robot):
    def __init__(self, enable_recording):
        super(CaptureTester, self).__init__()
        self.logitech = Camera("logitech")
        self.ps3eye = Camera("ps3eye", enabled=False)

        self.start_time = time.time()

        super(CaptureTester, self).__init__()

    def start(self):
        logitech_name = "%s%s.avi" % (self.get_path_info("file name no extension"), self.logitech.name)
        ps3eye_name = "%s%s.avi" % (self.get_path_info("file name no extension"), self.ps3eye.name)
        directory = self.get_path_info("input dir")

        if self.is_live:
            status = self.logitech.launch_camera(
                logitech_name, directory, self.logger.is_open(), capture_number=1
            )
            if status is not None:
                return status

            status = self.ps3eye.launch_camera(
                ps3eye_name, directory, self.logger.is_open(), capture_number=2
            )
            if status is not None:
                return status
        else:
            self.logitech.launch_video(logitech_name, directory)
            self.ps3eye.launch_video(ps3eye_name, directory)

        self.start_time = time.time()

    def loop(self):
        if self.logitech.get_frame(self.dt()) is None:
            return "exit"

        if self.ps3eye.get_frame(self.dt()) is None:
            return "exit"

        self.logitech.show_frame()
        self.ps3eye.show_frame()
        key = self.logitech.key_pressed()
        if key == 'q' or key == -2:
            return "done"

    def close(self, reason):
        self.logitech.close()
        self.ps3eye.close()


class PipelineTest:
    def __init__(self, use_pipeline):
        self.use_pipeline = use_pipeline
        self.pipeline = Pipeline()

        self.mac_cam = Camera("mac")

        if self.use_pipeline:
            self.pipeline.start()
        else:
            self.mac_cam.launch_camera(None, None, False, 0)

    def show(self):
        if self.use_pipeline:
            # self.pipeline.put(self.mac_cam.get_frame())
            with self.pipeline.pipeline_lock:
                if not self.pipeline.results_queue.empty():
                    print("2", self.pipeline.results_queue.empty())
                    cv2.imshow("pipeline", self.pipeline.results_queue.get())
                    # self.mac_cam.show_frame(frame)

        else:
            frame = self.mac_cam.get_frame()
            self.mac_cam.show_frame(cv2.medianBlur(frame, 111))


def run(live):
    video_tester = CaptureTester(False)

    if live:
        runner = RobotRunner(video_tester, log_data=False)
        runner.run()
    else:
        simulator = RobotSimulator("18;54", "2017_Mar_12", video_tester)
        simulator.run()


def pipeline():
    test = PipelineTest(True)
    while True:
        test.show()


run(True)
# pipeline()
