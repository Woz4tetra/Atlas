"""
RobotInterfaceSimulator imitates RobotInterface except its data source is a log file.
"""

from atlasbuggy.robot.object import RobotObject
from atlasbuggy.robot.collection import RobotObjectCollection
from atlasbuggy.interface.base import BaseInterface
from atlasbuggy.files import logfile
from atlasbuggy.robot.errors import *


class RobotSimulator(BaseInterface):
    def __init__(self, file_name, directory, robot, start_index=0, end_index=-1, debug_enabled=False):
        """
        :param file_name: log file name or number
        :param directory: directory to search in
        :param start_index:
        :param end_index:
        :param robot:
        """

        self.parser = logfile.Parser(file_name, directory, start_index, end_index)

        print("Using file:", self.parser.full_path)
        super(RobotSimulator, self).__init__(robot, debug_enabled, "Simulator")

        for robot_object in self.robot.objects.values():
            robot_object.is_live = False

        self.close_called = False
        self.current_index = 0

    def _start(self):
        self.robot.start()

    def _should_run(self):
        return self.parser.index < self.parser.end_index - 1

    def _update(self):
        if self.robot.is_paused:
            return

        line = self.parser.next()
        if line is None:
            return
        self.current_index = line[0]
        packet_type = line[1]
        self.robot.current_timestamp = line[2]
        self.robot.current_whoiam = line[3]
        self.robot.current_packet = line[4]

        if packet_type == "error" or packet_type == "debug":
            if self.debug_enabled:
                print(self.robot.current_packet)
            return
        else:
            self.robot.ids_received.add(self.robot.current_whoiam)

        if self.robot.current_whoiam in self.robot.packets_received:
            self.robot.packets_received[self.robot.current_whoiam] += 1
        else:
            self.robot.packets_received[self.robot.current_whoiam] = 0

        status = self._deliver_packet(self.robot.current_timestamp, self.robot.current_whoiam, self.robot.current_packet,
                                      packet_type)
        if status is not None:
            return status
        status = self._received(self.robot.dt(), self.robot.current_whoiam, self.robot.current_packet, packet_type)
        if status is not None:
            return status

    def _deliver_packet(self, timestamp, whoiam, packet, packet_type):
        if whoiam in self.robot.objects.keys() and packet_type == "object":
            if timestamp is None:
                if self.robot.objects[whoiam].enabled:
                    if isinstance(self.robot.objects[whoiam], RobotObject):
                        if self.robot.objects[whoiam].receive_first(packet) is not None:
                            return "exit"
                    elif isinstance(self.robot.objects[whoiam], RobotObjectCollection):
                        if self.robot.objects[whoiam].receive_first(whoiam, packet) is not None:
                            return "exit"
                    return
            else:
                try:
                    if self.robot.objects[whoiam].enabled:
                        if isinstance(self.robot.objects[whoiam], RobotObject):
                            if self.robot.objects[whoiam].receive(timestamp, packet) is not None:
                                return "exit"
                        elif isinstance(self.robot.objects[whoiam], RobotObjectCollection):
                            if self.robot.objects[whoiam].receive(timestamp, whoiam, packet) is not None:
                                return "exit"

                    if whoiam in self.robot.linked_functions:
                        self.robot.ids_used.add(whoiam)
                        status = self.robot.linked_functions[whoiam](timestamp, packet, packet_type)
                        if status is not None:
                            return "exit"
                except KeyboardInterrupt:
                    return "exit"
                except BaseException as error:
                    self._debug_print("RobotObject's receive signalled an error")
                    self._close("error")
                    raise RobotObjectReceiveError(whoiam, packet)

    def _received(self, timestamp, whoiam, packet, packet_type):
        try:
            status = self.robot.received(timestamp, whoiam, packet, packet_type)
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
            status = self.robot.loop()
            if status is not None:
                self._debug_print("loop signalled to exit")
                return status
        except BaseException as error:
            self._debug_print("_main_loop signalled an error")
            raise LoopSignalledError(error)

    def _close(self, reason=""):
        if not self.close_called:
            self.close_called = True
            if self.robot.ids_received != self.robot.ids_used:
                if len(self.robot.ids_received - self.robot.ids_used) > 0:
                    print("Warning, IDs unused in simulator:")
                    for id in self.robot.ids_received - self.robot.ids_used:
                        print("\t", id)
                if len(self.robot.ids_used - self.robot.ids_received) > 0:
                    print("Warning, IDs not in log file:")
                    for id in self.robot.ids_used - self.robot.ids_received:
                        print("\t", id)
            self.robot.close(reason)

    def num_received(self, arg):
        if isinstance(arg, RobotObject):
            return self.robot.packets_received[arg.whoiam]
        elif isinstance(arg, RobotObjectCollection):
            packets = 0
            for whoiam in arg.whoiam_ids:
                packets += self.robot.packets_received[whoiam]
            return packets
        else:
            return self.robot.packets_received[arg]
