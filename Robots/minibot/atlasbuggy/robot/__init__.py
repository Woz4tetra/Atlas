from atlasbuggy.robot.errors import *
from atlasbuggy.robot.clock import ReoccuringEvent, DelayedEvent
from atlasbuggy.robot.object import RobotObject
from atlasbuggy.robot.collection import RobotObjectCollection


class Robot:
    def __init__(self, *robot_objects):
        """
        :param robot_objects: instances of atlasbuggy.robot.object.RobotObject or
            atlasbuggy.robot.object.RobotObjectCollection
        """
        self.objects = {}  # all current robot object, key: whoiam, value: RobotObject
        self.linked_functions = {}  # user linked callback functions
        self.reoccuring_functions = []
        self.delayed_functions = []

        self.inactive_ids = set()  # disabled whoiam IDs
        self.ids_used = set()  # IDs used by the user
        self.ids_received = set()  # IDs received from a log or serial port

        self.packets_received = {}  # number of packets, key: whoiam, value: int

        self.current_packet = ""  # most recently received packet
        self.current_whoiam = ""  # most recently received whoiam
        self.current_timestamp = None  # most recently received timestamp

        self.is_paused = False  # for simulations. The robot can be paused (see self.pause)

        self.is_live = None
        self.debug_enabled = None

        # for runners
        self.logger = None  # Instance of atlasbuggy.files.logfile.Logger (None if is_live is False)
        self.parser = None  # Instance of atlasbuggy.files.logfile.Parser (None if is_live is True)
        self.queue_len = 0  # The number of packets on the packet queue
        self.joystick = None  # instance of BuggyJoystick

        # initialize self.robot_objects
        for robot_object in robot_objects:
            if isinstance(robot_object, RobotObject):
                if robot_object.enabled:  # only add object if it's enabled
                    self.objects[robot_object.whoiam] = robot_object
                else:
                    # add to inactive IDs if disabled
                    self.inactive_ids.add(robot_object.whoiam)

            elif isinstance(robot_object, RobotObjectCollection):
                if robot_object.enabled:  # only add object if it's enabled
                    for whoiam in robot_object.whoiam_ids:
                        self.objects[whoiam] = robot_object
                else:
                    # add to inactive IDs if disabled
                    for whoiam in robot_object.whoiam_ids:
                        self.inactive_ids.add(whoiam)

            else:
                raise RobotObjectInitializationError(
                    "Object passed is not valid:", repr(robot_object))

    def __getitem__(self, item):
        return self.objects[item]

    def dt(self):
        """
        Time since the start of the program. Uses the timestamps of received packets
        :return: 0.0 if the interface hasn't started, >0.0 otherwise
        """
        return 0.0 if self.current_timestamp is None else self.current_timestamp

    def start(self):
        """
        Events to be run when the interface starts (receive_first has been for all enabled robot objects)
        :return: None if ok, "error", "exit", or "done" if the program should exit
        """
        pass

    def received(self, timestamp, whoiam, packet, packet_type):
        """
        Events to be run when the interface receives any packet
        :return: None if ok, "error", "exit", or "done" if the program should exit
        """
        pass

    def loop(self):
        """
        Events to be run on a loop
        :return: None if ok, "error", "exit", or "done" if the program should exit
        """
        pass

    def close(self, reason):
        """
        Events to be run when the interface closes
        :param reason: "error", "exit", or "done"
            error - an error was thrown
            exit - something requested an premature exit
            done - something signalled to the program is done
        """
        pass

    def record(self, tag, string):
        """
        Record a string to the log if enabled

        :param tag: similar to a whoiam ID (make sure this doesn't overlap with any robot objects!
        :param string: similar to a packet. Data to record
        """
        if self.logger is not None:
            self.logger.record(self.dt(), tag, string, "user")

        if self.is_live is None:
            print("Warning! Robot has not been started by a robot runner. Ignoring record. "
                  "Put record statements in 'start', 'received', 'loop', or 'close'")

    def pause(self):
        """
        Sets self.is_paused to True. Only used in simulations
        """
        self.is_paused = True

    def unpause(self):
        self.is_paused = False

    def toggle_pause(self):
        self.is_paused = not self.is_paused

    def _get_whoiam(self, arg):
        if isinstance(arg, RobotObject):
            return arg.whoiam
        elif isinstance(arg, RobotObjectCollection):
            return arg.whoiam_ids
        else:
            return arg

    def get_path_info(self, arg):
        if self.logger is not None:
            info_source = self.logger
        elif self.parser is not None:
            info_source = self.parser
        else:
            return None

        if arg == "input name":
            return info_source.input_name
        elif arg == "input dir":
            return info_source.input_dir
        elif arg == "file types":
            return info_source.file_types
        elif arg == "default dir":
            return info_source.default_dir
        elif arg == "abs directory":
            return info_source.directory
        elif arg == "file name":
            return info_source.file_name
        elif arg == "file name no extension":
            return info_source.file_name_no_ext
        elif arg == "full path":
            return info_source.full_path

    def link_object(self, arg, callback_fn):
        """
        Link a callback function
            example: self.link(self.robot_object, self.some_function)
        :param arg: a whoiam ID or robot object
        :param callback_fn: a function reference
        """
        whoiam = self._get_whoiam(arg)
        self.linked_functions[whoiam] = callback_fn

    def link_reoccuring(self, repeat_time, callback_fn, *args, current_time=None):
        self.reoccuring_functions.append(ReoccuringEvent(repeat_time, current_time, callback_fn, args))

    def delay_function(self, delay_time, current_time, callback_fn, *args):
        self.delayed_functions.append(DelayedEvent(delay_time, current_time, callback_fn, args))

    def did_receive(self, arg):
        """
        Check if the previously received packet matches the parsed argument.
        Adds the arg's whoiam to ids_used

        :param arg: whoiam ID or robot object
        :return: True or False if the arg matches the most recent packet
        """
        if isinstance(arg, RobotObject):
            self.ids_used.add(arg.whoiam)
            return arg.whoiam == self.current_whoiam
        elif isinstance(arg, RobotObjectCollection):
            for whoiam in arg.whoiam_ids:
                self.ids_used.add(whoiam)
                return whoiam == self.current_whoiam
        else:
            self.ids_used.add(arg)
            return arg == self.current_whoiam

    def num_received(self, arg):
        """
        Number of packets received from the given argument

        :param arg: whoiam ID or robot object
        :return: number of packets received with the arg's whoiam
        """
        if isinstance(arg, RobotObject):
            return self.packets_received[arg.whoiam]
        elif isinstance(arg, RobotObjectCollection):
            packets = 0
            for whoiam in arg.whoiam_ids:
                packets += self.packets_received[whoiam]
            return packets
        else:
            return self.packets_received[arg]

    def debug_print(self, *values, ignore_flag=False):
        string = "[%s] %s" % (self.__class__.__name__, " ".join([str(x) for x in values]))
        if self.is_live:
            self.logger.record(self.current_timestamp, self.__class__.__name__, string, "debug")

        if self.debug_enabled or ignore_flag:
            print(string)
