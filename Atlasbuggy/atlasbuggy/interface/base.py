# a common subclass for RobotRunner and RobotSimulator. Contains common methods and properties


class BaseInterface:
    def __init__(self, robot, debug_enabled, debug_name):
        """
        :param robot: a subclass instance of the atlasbuggy.robot.Robot class
        :param debug_enabled: print debug messages
        :param debug_name: name to put in the debug messages when the message comes from the interface
            as opposed to a robot port
        """
        self.robot = robot

        self.debug_enabled = debug_enabled
        self.debug_name = debug_name

        self.exit_thrown = False

        self.input_name = ""
        self.input_dir = ""
        self.directory = ""
        self.file_name = ""
        self.full_path = ""
        self.file_name_no_ext = ""

    def run(self, loop=None):
        """
        run skeleton for the interface. Calls the interface's wrapper methods:
            _start, _should_run, _update, _loop, _close
        """

        # initialization
        self._start()
        try:
            # should the interface continue?
            while self._should_run() and not self.exit_thrown:
                # get incoming packets and update appropriate methods
                status = self._update()
                if status is not None:
                    self._close(status)
                    return

                # call this method no matter what (think of Arduino's loop)
                status = self._loop()
                if status is not None:
                    self._close(status)
                    return

                try:
                    if loop is not None:
                        loop()
                except BaseException as error:
                    self._debug_print("Closing from external loop")
                    self._close("error")
                    raise error

        except KeyboardInterrupt:
            pass

        self._close("done")

    def get_path(self):
        return self.file_name_no_ext, self.input_dir

    # ----- internal methods -----

    def _start(self):
        pass

    def _loop(self):
        pass

    def _update(self):
        pass

    def _close(self, reason):
        pass

    def _should_run(self):
        pass

    def exit(self):
        """
        Exit the interface (use if interface is on a separate thread)
        """
        self.exit_thrown = True

    def _debug_print(self, *values, ignore_flag=False):
        """
        print if debug_enabled is True (can be overridden by ignore_flag)

        :param values: multiple values can be passed so long as they can be converted to strings.
            The values are joined by " " by default
        :param ignore_flag: override debug_enabled
        """
        if self.debug_enabled or ignore_flag:
            string = " ".join([str(x) for x in values])
            print("[%s] %s" % (self.debug_name, string))
