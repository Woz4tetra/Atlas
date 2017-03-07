class BaseInterface:
    def __init__(self, robot, debug_enabled, debug_name):
        self.robot = robot

        self.debug_enabled = debug_enabled

        self.debug_name = debug_name

    def run(self):
        self._start()
        try:
            while self._should_run():
                status = self._update()
                if status is not None:
                    self._close(status)
                    return

                status = self._loop()
                if self._loop() is not None:
                    self._close(status)
                    return

                self._extra_events()
        except KeyboardInterrupt:
            pass

        self._close("done")

    # ----- internal methods -----

    def _start(self):
        pass

    def _loop(self):
        pass

    def _update(self):
        pass

    def _extra_events(self):
        pass

    def _close(self, reason):
        pass

    def _should_run(self):
        pass

    def _debug_print(self, *strings, ignore_flag=False):
        if self.debug_enabled or ignore_flag:
            string = " ".join([str(x) for x in strings])
            print("[%s] %s" % (self.debug_name, string))
