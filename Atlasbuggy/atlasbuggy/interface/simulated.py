"""
RobotInterfaceSimulator imitates RobotInterface except its data source is a log file.
"""

import time

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
        self.is_paused = False

    def _start(self):
        self.start()

    def _should_run(self):
        return self.parser.index < self.parser.end_index - 1

    def pause(self):
        self.is_paused = True

    def unpause(self):
        self.is_paused = False

    def toggle_pause(self):
        self.is_paused = not self.is_paused

    def _update(self):
        if self.is_paused:
            return

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

        status = self._deliver_packet(self.dt(), self.current_whoiam, self.current_packet, packet_type)
        if status is not None:
            return status
        status = self._received(self.dt(), self.current_whoiam, self.current_packet, packet_type)
        if status is not None:
            return status

    def _deliver_packet(self, timestamp, whoiam, packet, packet_type):
        if whoiam in self.objects.keys() and packet_type == "object":
            if timestamp == logfile.no_timestamp:
                if self.objects[whoiam].enabled:
                    if isinstance(self.objects[whoiam], RobotObject):
                        if self.objects[whoiam].receive_first(packet) is not None:
                            return "exit"
                    elif isinstance(self.objects[whoiam], RobotObjectCollection):
                        if self.objects[whoiam].receive_first(whoiam, packet) is not None:
                            return "exit"
                    return
            else:
                try:
                    if self.objects[whoiam].enabled:
                        if isinstance(self.objects[whoiam], RobotObject):
                            if self.objects[whoiam].receive(timestamp, packet) is not None:
                                return "exit"
                        elif isinstance(self.objects[whoiam], RobotObjectCollection):
                            if self.objects[whoiam].receive(timestamp, whoiam, packet) is not None:
                                return "exit"
                except KeyboardInterrupt:
                    return "exit"
                except BaseException as error:
                    self._debug_print("RobotObject's receive signalled an error")
                    self._close("error")
                    raise RobotObjectReceiveError(whoiam, packet)

            if whoiam in self._linked_functions:
                status = self._linked_functions[whoiam](timestamp, packet, packet_type)
                if status is not None:
                    return "exit"

    def _received(self, timestamp, whoiam, packet, packet_type):
        try:
            status = self.received(timestamp, whoiam, packet, packet_type)
            if status is not None:
                self._debug_print(
                    "user's received method signalled to exit. whoiam ID: '%s', packet: %s" % (whoiam, repr(packet)))
                return "exit"
        except KeyboardInterrupt:
            return "exit"
        except BaseException as error:
            self._debug_print("Closing all from simulated _update")
            self._close("error")
            raise PacketReceivedError(error)

    def _loop(self):
        try:
            status = self.loop()
            if status is not None:
                self._debug_print("loop signalled to exit")
                return status
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
