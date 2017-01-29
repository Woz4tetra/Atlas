import sys
import time

from autobuggy.vision.video import Video
# from autobuggy import project
from pipeline import Pipeline


class PipelineTest:
    def __init__(self):
        # project.set_project_dir("rccar")
        if len(sys.argv) == 2:
            file_name = sys.argv[1]
            directory = None
        elif len(sys.argv) == 3:
            file_name = sys.argv[1]
            directory = sys.argv[2]
        else:
            file_name = 0
            directory = "rc_car"
        try:
            file_name = int(file_name)
        except ValueError:
            pass

        self.width = 480
        self.height = 320

        self.show_original = False

        self.capture = Video(file_name, directory)
        self.pipeline = Pipeline(self.width, self.height, True)

        self.time_start = time.time()

    def update_keys(self):
        key = self.capture.key_pressed()

        if key == 'q' or key == "esc":
            print("quitting...")
            return False  # exit program
        elif key == ' ':
            if self.capture.paused:
                print("%0.4fs: ...Video unpaused" % (
                    time.time() - self.time_start))
            else:
                print("%0.4fs: Video paused..." % (
                    time.time() - self.time_start))
            self.capture.paused = not self.capture.paused
        elif key == 's':
            self.capture.save_frame()
        elif key == 'v':
            if not self.capture.is_recording:
                self.capture.start_recording()
            else:
                self.capture.stop_recording()
        elif key == 'o':
            self.show_original = not self.show_original
        elif key == 'right':
            self.capture.increment_frame()
        elif key == 'left':
            self.capture.decrement_frame()

        if not self.capture.paused:
            self.capture.show_frame()

        return True  # don't exit program

    def run(self):
        while True:
            if not self.capture.paused or self.capture.slider_moved():
                if self.capture.get_frame() is None:
                    break
                if not self.show_original:
                    frame = self.pipeline.update(self.capture)
                    self.capture.show_frame(frame)
                else:
                    self.capture.show_frame()

            if not self.update_keys():
                break

PipelineTest().run()
