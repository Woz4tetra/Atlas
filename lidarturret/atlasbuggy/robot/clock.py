"""
Keeps loops running at a consistent time. This class will notify you if the loop is falling behind
"""
import time


class Clock:
    def __init__(self, loops_per_second):
        """
        :param loops_per_second: similar to frames per second
        """
        self.loop_time = 0
        if loops_per_second is not None:
            self.seconds_per_loop = 1 / loops_per_second
        else:
            self.seconds_per_loop = None

        self.current_time = 0
        self.time_diff = 0
        self.offset = 0

        self.on_time = True

    def start(self, start_time=None):
        """
        Initialize the clock's time. start_time is used to provide a consist start time
        :param start_time:
        :return: None
        """
        if start_time is None:
            start_time = time.time()
        self.loop_time = start_time
        self.current_time = start_time

    def update(self):
        """
        pause the process if the elapsed time is greater than loops_per_second.
        :return: True if the process was paused. False if the loop ran too slowly
        """
        if self.seconds_per_loop is None:
            return True

        self.current_time = time.time()
        if self.current_time != self.loop_time:
            self.time_diff = self.current_time - self.loop_time
            self.offset = self.seconds_per_loop - self.time_diff

            if self.offset > 0:
                self.on_time = True
                time.sleep(self.offset)
            else:
                self.on_time = False

            self.loop_time = time.time()

            return self.offset > 0
        else:
            return False
