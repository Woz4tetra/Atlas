"""
This module handles all interfacing with serial ports and passing data from them.
The idea is every microcontroller gets one corresponding robot object and port (which runs
on its own process).
What data you send over serial is up to you. There is no strict packet protocol.
"""

from atlasbuggy.robot.errors import *
from atlasbuggy.robot.object import RobotObject
from atlasbuggy.robot.collection import RobotObjectCollection


class Robot:
    def __init__(self, *robot_objects):
        self.linked_functions = {}
        self.objects = {}

        self.inactive_ids = set()
        self.ids_used = set()
        self.ids_received = set()

        self.packets_received = {}

        self.current_packet = ""
        self.current_whoiam = ""
        self.current_timestamp = None

        self.is_paused = False

        self.logger = None
        self.queue_len = 0

        self.joystick = None

        for robot_object in robot_objects:
            if isinstance(robot_object, RobotObject):
                if robot_object.enabled:
                    self.objects[robot_object.whoiam] = robot_object
                else:
                    self.inactive_ids.add(robot_object.whoiam)

            elif isinstance(robot_object, RobotObjectCollection):
                if robot_object.enabled:
                    for whoiam in robot_object.whoiam_ids:
                        self.objects[whoiam] = robot_object
                else:
                    for whoiam in robot_object.whoiam_ids:
                        self.inactive_ids.add(whoiam)

            else:
                raise RobotObjectInitializationError(
                    "Object passed isn't a RobotObject or RobotObjectCollection:", repr(robot_object))

    def dt(self):
        return 0.0 if self.current_timestamp is None else self.current_timestamp

    def start(self):
        pass

    def received(self, timestamp, whoiam, packet, packet_type):
        pass

    def loop(self):
        pass

    def close(self, reason):
        pass

    def record(self, tag, string):
        if self.logger is not None:
            self.logger.record(self.dt(), tag, string, "user")

    def pause(self):
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
        whoiam = self._get_whoiam(arg)
        self.linked_functions[whoiam] = callback_fn

    def did_receive(self, arg):
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
