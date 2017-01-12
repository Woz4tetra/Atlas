import time


class Clock:
    def __init__(self, loops_per_second):
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
        if start_time is None:
            start_time = time.time()
        self.loop_time = start_time
        self.current_time = start_time

    def update(self):
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
