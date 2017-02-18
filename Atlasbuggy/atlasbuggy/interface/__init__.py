from atlasbuggy.robot.errors import *
from atlasbuggy.robot.object import RobotObject
from atlasbuggy.robot.collection import RobotObjectCollection


class BaseInterface:
    def __init__(self, robot_objects, debug_enabled):
        self.objects = {}

        self.inactive_ids = set()
        self.ids_used = set()
        self.ids_received = set()
        self.packets_received = {}

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

        self.debug_enabled = debug_enabled

        self.current_packet = ""
        self.current_whoiam = ""
        self.current_timestamp = 0

        self.close_called = False

    def start(self):
        pass

    def received(self, timestamp, whoiam, packet, packet_type):
        pass

    def loop(self):
        pass

    def close(self, reason):
        pass

    def run(self):
        self._start()
        try:
            while self._should_run():
                status = self._update()
                if status is not None:
                    self._close(status)
                    return

                status = self._loop()
                if self._loop() is False:
                    self._close(status)
                    return

                self._extra_events()
        except KeyboardInterrupt:
            pass

        self._close("done")

    def dt(self):
        pass

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
            print("[Interface] %s" % string)
