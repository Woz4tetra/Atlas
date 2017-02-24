"""
RobotInterfaceSimulator imitates RobotInterface except its data source is a log file.
"""

from atlasbuggy.robot.object import RobotObject
from atlasbuggy.robot.collection import RobotObjectCollection
from atlasbuggy.interface import BaseInterface
from atlasbuggy.files import logfile
from atlasbuggy.robot.errors import *


class SimulatedRobot(BaseInterface):
    def __init__(self, file_name, directory, *robot_objects, start_index=0, end_index=-1, debug_enabled=False):
        """
        :param file_name: log file name or number
        :param directory: directory to search in
        :param start_index:
        :param end_index:
        :param robot_objects:
        """

        self.parser = logfile.Parser(file_name, directory, start_index, end_index)

        print("Using file:", self.parser.full_path)
        super(SimulatedRobot, self).__init__(robot_objects, debug_enabled=debug_enabled)

        self._dt = 0
        self.current_index = 0

    def _start(self):
        self.start()

    def _should_run(self):
        return self.parser.index < self.parser.end_index - 1

    def _update(self):
        line = self.parser.next()
        if line is None:
            return
        self.current_index, packet_type, self._dt, self.current_whoiam, self.current_packet = line

        if packet_type == "error" or packet_type == "debug":
            print(self.current_packet)
            return
        else:
            self.ids_received.add(self.current_whoiam)

        if self.current_whoiam in self.packets_received:
            self.packets_received[self.current_whoiam] += 1
        else:
            self.packets_received[self.current_whoiam] = 0

        if self.current_whoiam in self.objects.keys() and packet_type == "object":
            if self._dt == logfile.no_timestamp:
                if self.objects[self.current_whoiam].enabled:
                    if isinstance(self.objects[self.current_whoiam], RobotObject):
                        self.objects[self.current_whoiam].receive_first(self.current_packet)
                    elif isinstance(self.objects[self.current_whoiam], RobotObjectCollection):
                        self.objects[self.current_whoiam].receive_first(self.current_whoiam, self.current_packet)
                    return
            else:
                try:
                    if self.objects[self.current_whoiam].enabled:
                        if isinstance(self.objects[self.current_whoiam], RobotObject):
                            self.objects[self.current_whoiam].receive(self._dt, self.current_packet)
                        elif isinstance(self.objects[self.current_whoiam], RobotObjectCollection):
                            self.objects[self.current_whoiam].receive(self._dt, self.current_whoiam,
                                                                      self.current_packet)
                except KeyboardInterrupt:
                    return "exit"
                except BaseException as error:
                    self._debug_print("RobotObject's receive signalled an error")
                    self._close("error")
                    raise RobotObjectReceiveError(self.current_whoiam, self.current_packet)

        try:
            status = self.received(self._dt, self.current_whoiam, self.current_packet, packet_type)
            if status is not None:
                self._debug_print(
                    "user's received method signalled to exit. whoiam ID: '%s', packet: %s" % (
                        self.current_whoiam, repr(self.current_packet)))
                return "exit"
        except KeyboardInterrupt:
            return "exit"
        except BaseException as error:
            self._debug_print("Closing all from simulated _update")
            self._close("error")
            raise PacketReceivedError(error)

    def _loop(self):
        try:
            if self.loop() is False:
                self._debug_print("loop signalled to exit")
                return "error"
        except BaseException as error:
            self._debug_print("_main_loop signalled an error")
            raise LoopSignalledError(error)

    def _close(self, reason=""):
        if not self.close_called:
            self.close_called = True
            if self.ids_received != self.ids_used:
                if len(self.ids_received - self.ids_used) > 0:
                    print("Warning, IDs unused in simulator:")
                    for id in self.ids_received - self.ids_used:
                        print("\t", id)
                if len(self.ids_used - self.ids_received) > 0:
                    print("Warning, IDs not in log file:")
                    for id in self.ids_used - self.ids_received:
                        print("\t", id)
            self.close(reason)

    def dt(self):
        return self._dt

    def num_received(self, arg):
        if isinstance(arg, RobotObject):
            return self.packets_received[arg.whoiam]
        elif isinstance(arg, RobotObjectCollection):
            packets = 0
            for whoiam in arg.whoiam_ids:
                packets += self.packets_received[whoiam]
            return packets
        else:
            return self.packets_received[arg]
