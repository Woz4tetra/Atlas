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
        :param file_name: log file to use for simulation
        :param directory: directory to search in
        :param start_index: line number to start simulating from
        :param end_index: line number to end simulation on
        :param robot: a subclass instance of the atlasbuggy.robot.Robot class
        """

        if start_index != 0:
            print("Starting log from", start_index)
        if end_index != -1:
            print("Ending log on", end_index)

        # read the log file
        self.parser = logfile.Parser(file_name, directory, start_index, end_index)

        print("Using file:", self.parser.full_path)
        super(RobotSimulator, self).__init__(robot, debug_enabled, "Simulator")

        # tell robot objects it is simulated so command queues don't build up unnecessarily
        for robot_object in self.robot.objects.values():
            robot_object.is_live = False
        self.robot.is_live = False

        self.robot.parser = self.parser

        self.current_index = 0

    def _start(self):
        while self.robot.current_timestamp is None:
            self._update()
        self.robot.start()

    def _should_run(self):
        return self.parser.index < self.parser.end_index - 1

    def _update(self):
        if self.robot.is_paused:
            return

        # parse the current line, return the contents
        line = self.parser.next()
        if line is None:  # if line was not parsed correctly
            return
        self.current_index = line[0]
        packet_type = line[1]  # packet types: object, command, user, error, debug
        self.robot.current_timestamp = line[2]
        self.robot.current_whoiam = line[3]
        self.robot.current_packet = line[4]

        # print errors and debug prints (if enabled)
        if packet_type == "error" or (packet_type == "debug" and self.debug_enabled):
            print("\t%s" % self.robot.current_packet)
            return
        else:
            self.robot.ids_received.add(self.robot.current_whoiam)

        # Add to the packet counter dictionary
        if self.robot.current_whoiam in self.robot.packets_received:
            self.robot.packets_received[self.robot.current_whoiam] += 1
        else:
            self.robot.packets_received[self.robot.current_whoiam] = 0

        # deliver packet to the corresponding robot object
        status = self._deliver_packet(
            self.robot.current_timestamp, self.robot.current_whoiam, self.robot.current_packet, packet_type
        )
        if status is not None:
            return status

        # call the simulated robot's receive method
        status = self._received(self.robot.dt(), self.robot.current_whoiam, self.robot.current_packet, packet_type)
        if status is not None:
            return status

    def _deliver_packet(self, timestamp, whoiam, packet, packet_type):
        """
        Deliver packet to the corresponding robot object

        :param timestamp: time received since start of the program
        :param whoiam: whoiam ID of the packet
        :param packet: a string containing the received packet
        :param packet_type: packet types: object, command, user, error, debug
            object - received from microcontroller
            command - sent to microcontroller
            user - packets recorded with robot.record
            error - error messages and tracebacks
            debug - debug messages from ports and interface
        :return: status: error, exit, done or None
        """
        if whoiam in self.robot.objects.keys() and packet_type == "object":
            if timestamp is None:
                if self.robot.objects[whoiam].enabled:
                    # call robot object's receive_first method
                    if isinstance(self.robot.objects[whoiam], RobotObject):
                        status = self.robot.objects[whoiam].receive_first(packet)
                    elif isinstance(self.robot.objects[whoiam], RobotObjectCollection):
                        status = self.robot.objects[whoiam].receive_first(whoiam, packet)
                    else:
                        status = None

                    # Return. Don't call robot's receive method
                    return status
            else:
                try:
                    if self.robot.objects[whoiam].enabled:
                        # call robot object's receive method
                        if isinstance(self.robot.objects[whoiam], RobotObject):
                            status = self.robot.objects[whoiam].receive(timestamp, packet)
                        elif isinstance(self.robot.objects[whoiam], RobotObjectCollection):
                            status = self.robot.objects[whoiam].receive(timestamp, whoiam, packet)
                        else:
                            status = None

                        if status is not None:
                            return status

                    # call user's linked functions
                    if whoiam in self.robot.linked_functions:
                        self.robot.ids_used.add(whoiam)
                        status = self.robot.linked_functions[whoiam](timestamp, packet, packet_type)
                        if status is not None:
                            return status
                except KeyboardInterrupt:
                    return "done"
                except BaseException as error:
                    self._debug_print("RobotObject's receive signalled an error")
                    self._close("error")
                    raise RobotObjectReceiveError(whoiam, packet)

    def _received(self, timestamp, whoiam, packet, packet_type):
        """
        Call robot's received method when a packet is received

        :param timestamp: time received since start of the program
        :param whoiam: whoiam ID of the packet
        :param packet: a string containing the received packet
        :param packet_type: packet types: object, command, user, error, debug
        :return: status: error, exit, done or None
        """
        try:
            status = self.robot.received(timestamp, whoiam, packet, packet_type)
            if status is not None:
                self._debug_print(
                    "user's received method signalled to exit. whoiam ID: '%s', packet: %s" % (whoiam, repr(packet)))
                return status
        except KeyboardInterrupt:
            return "done"
        except BaseException as error:
            self._debug_print("Closing all from simulated _update")
            self._close("error")
            raise PacketReceivedError(error)

    def _loop(self):
        """
        Call robot's loop method
        :return: status: error, exit, done or None
        """
        try:
            status = self.robot.loop()
            if status is not None:
                self._debug_print("loop signalled to exit")
                return status
        except BaseException as error:
            self._debug_print("_main_loop signalled an error")
            raise LoopSignalledError(error)

    def _close(self, reason):
        """
        Close the simulator. Notify the user of unused whoiam IDs
        :param reason: reason for closing: error, exit, done or None
        """
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
