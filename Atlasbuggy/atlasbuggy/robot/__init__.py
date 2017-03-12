from atlasbuggy.robot.errors import *
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

        self.inactive_ids = set()  # disabled whoiam IDs
        self.ids_used = set()  # IDs used by the user
        self.ids_received = set()  # IDs received from a log or serial port

        self.packets_received = {}  # number of packets, key: whoiam, value: int

        self.current_packet = ""  # most recently received packet
        self.current_whoiam = ""  # most recently received whoiam
        self.current_timestamp = None  # most recently received timestamp

        self.is_paused = False  # for simulations. The robot can be paused (see self.pause)

        # for runners
        self.logger = None  # Instance of atlasbuggy.files.logfile.Logger (None if unused)
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
                    "Object passed isn't a RobotObject or RobotObjectCollection:", repr(robot_object))

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

    def link(self, arg, callback_fn):
        """
        Link a callback function
            example: self.link(self.robot_object, self.some_function)
        :param arg: a whoiam ID or robot object
        :param callback_fn: a function reference
        """
        whoiam = self._get_whoiam(arg)
        self.linked_functions[whoiam] = callback_fn

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
