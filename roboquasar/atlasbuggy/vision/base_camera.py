from threading import Thread

from atlasbuggy.vision.capture import Capture


class BaseCamera(Capture):
    """A class for reading from any camera"""

    def __init__(self, width, height, window_name="BaseCamera",
                 enable_draw=True,
                 pipeline=None, update_fn=None, fn_params=None):
        """
        :param width: set a width for the capture
        :param height: set a height for the capture
        :param window_name: set a opencv window name
        :param enable_draw: whether the opencv window should be shown
            (boosts frames per second)
        :param pipeline: a class with a method named update. This class should
            parse the frame a return any useful data
        :param update_fn: the camera runs on a separate thread. Put any extra
            code to run in this function
        :param fn_params: parameters to pass to update_fn
        """
        super(BaseCamera, self).__init__(
            width, height, window_name, enable_draw, update_fn, fn_params
        )

        # initialize pipeline variables
        self.pipeline = pipeline
        self.analyzed_frame = None
        self.pipeline_results = {}
        self.thread = Thread(target=self.update, args=())

    def start(self):
        """start the thread to read frames from the video stream"""
        self.thread.start()

    def update(self):
        """Keep reading from the camera until self.stopped is True"""
        pass

    def get_frame(self):
        """Get the most recent frame read from the camera"""
        return self.frame

    def current_pos(self):
        return self.frame_num
