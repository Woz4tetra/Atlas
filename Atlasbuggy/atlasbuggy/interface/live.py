"""
This class manages all robot ports and pass data received to the corresponding robot objects.
"""

import pprint
import threading
import time
import traceback
from multiprocessing import Lock, Queue, Value

import serial
import serial.tools.list_ports

from atlasbuggy.files.logfile import Logger
from atlasbuggy.robot.clock import Clock
from atlasbuggy.robot.errors import *
from atlasbuggy.robot.collection import RobotObjectCollection
from atlasbuggy.robot.object import RobotObject
from atlasbuggy.robot.port import RobotSerialPort
from atlasbuggy.interface import BaseInterface


class LiveRobot(BaseInterface):
    loop_updates_per_second = 120
    port_updates_per_second = 1000

    def __init__(self, *robot_objects, joystick=None, log_data=True, log_name=None, log_dir=None, debug_prints=False):
        """
        :param robot_objects: subclasses of RobotObject
        :param joystick: A subclass instance of BuggyJoystick
        :param log_data: A boolean indicating whether or not the received data should be written to a logs file
        :param log_name: The name of the logs file. If None, it will be today's date and time
        :param log_dir: The directory of the logs file. If None, it will be today's date.
            See project.py for details
        :param debug_prints: Enable verbose prints
        :param updates_per_second: How quickly each port process should run.
        """

        super(LiveRobot, self).__init__(robot_objects, debug_prints)

        self.loop_ups = LiveRobot.loop_updates_per_second
        self.port_ups = LiveRobot.port_updates_per_second
        self.lag_warning_thrown = False  # prevents the terminal from being spammed
        self.current_whoiam = ""
        self.current_timestamp = 0
        self.prev_packet_info = [None, None, None]

        self.joystick = joystick

        self.logger = Logger(log_name, log_dir)
        if log_data:
            self.logger.open()
            print("Writing to:", self.logger.full_path)

        self.clock = Clock(self.loop_ups)
        self.start_time = 0

        # a pipe from all port processes to the main loop
        self.packet_queue = Queue()
        self.packet_counter = Value('i', 0)
        self.port_lock = Lock()

        # open all available ports using multithreading
        self.ports = {}
        threads = []

        self.duplicate_id_error = [False, None]
        for port_info in serial.tools.list_ports.comports():
            config_thread = threading.Thread(target=self._configure_port, args=(port_info, self.port_ups))
            threads.append(config_thread)
            config_thread.start()

        for thread in threads:
            thread.join()

        if self.duplicate_id_error[0]:
            self._close_ports("error")
            raise self._handle_error(
                RobotSerialPortWhoiamIdTaken("whoiam ID already being used by another port! It's possible "
                                             "the same code was uploaded for two boards.",
                                             self.prev_packet_info, self.duplicate_id_error[1]),
                traceback.format_stack()
            )

        for port_name, port in self.ports.items():
            if not port.configured:
                self._print_port_info(port)
                raise self._handle_error(
                    RobotSerialPortNotConfiguredError("Port not configured!", self.prev_packet_info, port),
                    traceback.format_stack()
                )

        self._check_objects()  # check that all objects are assigned a port
        self._check_ports()  # check that all ports are assigned an object

        for whoiam in self.ports.keys():
            self._debug_print("[%s] has ID '%s'" % (self.ports[whoiam].address, whoiam))

        if len(self.inactive_ids) > 0 and self.debug_enabled:
            self._debug_print("Ignored IDs:")
            for whoiam in self.inactive_ids:
                self._debug_print(whoiam)

        self._send_first_packets()  # distribute initialization packets

    def _start(self):
        self.start_time = time.time()
        self.clock.start(self.start_time)

        self._open_ports()
        try:
            self.start()  # call user's start method (empty by default)
        except BaseException as error:
            self._debug_print("Closing all from user's start")
            self._close_ports("error")
            raise self._handle_error(
                StartSignalledError("User's start method threw an exception"),
                traceback.format_stack()
            )

    def _loop(self):
        if self.joystick is not None:
            status = self.joystick.update()
            if status is not None:
                self._debug_print("Joystick signalled to exit")
                if status == "error":
                    self._close_ports("error")
                    raise self._handle_error(
                        LoopSignalledError("Joystick timed out!"),
                        traceback.format_stack()
                    )
                return status

        try:
            status = self.loop()
            if status is not None:
                self._debug_print("User's loop method signalled to exit")
                return status
        except BaseException as error:
            self._debug_print("_loop signalled an error")
            raise self._handle_error(
                LoopSignalledError(error),
                traceback.format_stack()
            )

    def _update(self):
        """
        dequeue all packets from packet_queue. Pass them to the corresponding robot objects
        :return: what packet_received returns (True or False signalled to exit or not)
        """
        with self.port_lock:
            while not self.packet_queue.empty():
                self.current_whoiam, self.current_timestamp, self.current_packet = self.packet_queue.get()
                self.packet_counter.value -= 1
                dt = self.current_timestamp - self.start_time

                status = self._deliver_packet(dt, self.current_whoiam, self.current_packet)
                if status is not None:
                    return status

                if self.logger.is_open():
                    self.logger.record(dt, self.current_whoiam, self.current_packet, "object")
                    self._record_debug_prints(dt, self.ports[self.current_whoiam])

                status = self._received(dt, self.current_whoiam, self.current_packet)
                if status is not None:
                    return status

    def _extra_events(self):
        self._send_commands()  # sends all commands in each robot object's command queue

        self.clock.update()  # maintain a constant loop speed
        if not self.lag_warning_thrown and self.dt() > 0.1 and not self.clock.on_time:
            print("Warning. Main loop is running slow.")
            self.lag_warning_thrown = True

    def _close(self, reason=""):
        self._debug_print("Closing all")

        self._close_log()
        self._close_ports(reason)

    def _should_run(self):
        """
        Using each robot port's is_running method, check if the processes are running properly
        An error will be thrown if not.

        :return: True if the ports are ok
        """
        for robot_port in self.ports.values():
            status = robot_port.is_running()
            if status < 1:
                self._debug_print("Closing all from _are_ports_active")
                self._close_ports("error")
                if status == 0:
                    raise self._handle_error(
                        RobotSerialPortNotConfiguredError(
                            "Port with ID '%s' isn't configured!" % robot_port.whoiam, self.prev_packet_info,
                            robot_port),
                        traceback.format_stack()
                    )
                elif status == -1:
                    raise self._handle_error(
                        RobotSerialPortSignalledExitError(
                            "Port with ID '%s' signalled to exit" % robot_port.whoiam, self.prev_packet_info,
                            robot_port),
                        traceback.format_stack()
                    )
        return True

    # ----- utility methods -----

    def dt(self):
        """
        Access the clock. This time uses the same reference as all other objects
        :return: time since the program's start in seconds
        """
        if self.start_time == 0:
            return None
        else:
            return time.time() - self.start_time

    def record(self, tag, string):
        """
        Record data not created by a robot object.

        :param tag: Unique tag similar to whoiam ID. Make sure these don't overlap with any robot_objects
        :param string: Similar to a packet. String data to record
        :return: None
        """
        self.logger.record(self.dt(), tag, string, "user")

    def queue_len(self):
        return self.packet_counter.value

    # ----- configuration methods -----

    def _configure_port(self, port_info, updates_per_second):
        """
        Initialize a serial port recognized by pyserial.
        Only devices that are plugged in should be recognized

        :param port_info: an instance of serial.tools.list_ports_common.ListPortInfo
        :param updates_per_second: how often the port should update
        :return: None
        """
        if port_info.vid is not None:
            port = RobotSerialPort(port_info, self.debug_enabled,
                                   self.packet_queue, self.port_lock, self.packet_counter, updates_per_second)
            self._debug_print("whoiam", port.whoiam)
            if port.whoiam in self.ports.keys():
                self.duplicate_id_error[0] = True
                self.duplicate_id_error[1] = port
            elif port.configured and (not port.abides_protocols or port.whoiam is None):
                self._debug_print("Warning! Port '%s' does not abide Atlasbuggy protocol!" % port.address)
                port.stop()
            elif not port.configured:
                self._debug_print("Port not configured! '%s'" % port.address)
            elif port.whoiam in self.inactive_ids:
                port.stop()
            else:
                self.ports[port.whoiam] = port

    def _check_objects(self):
        """
        Validate that all objects are assigned to ports. Throw RobotObjectNotFoundError otherwise
        :return: None
        """
        self._debug_print("Discovered ports:", list(self.ports.keys()))
        for whoiam in self.objects.keys():
            if whoiam not in self.ports.keys():
                self._close_ports("error")
                raise self._handle_error(
                    RobotObjectNotFoundError("Failed to assign robot object with ID '%s'" % whoiam),
                    traceback.format_stack()
                )

    def _check_ports(self):
        """
        Validate that all ports are assigned to objects. Throw RobotSerialPortUnassignedError otherwise
        :return: None
        """
        used_ports = {}
        for whoiam in self.ports.keys():
            if whoiam not in self.objects.keys():
                self._debug_print("Warning! Port ['%s', %s] is unused!" %
                                  (self.ports[whoiam].address, whoiam), ignore_flag=True)
            else:
                used_ports[whoiam] = self.ports[whoiam]
                object_baud = self.objects[whoiam].baud
                if object_baud is not None and object_baud != self.ports[whoiam].baud_rate:
                    self.ports[whoiam].change_rate(object_baud)
        self.ports = used_ports

    def _send_first_packets(self):
        """
        Send each port's first packet to the corresponding object if it isn't an empty string
        :return: None
        """
        status = True
        for whoiam in self.objects.keys():
            first_packet = self.ports[whoiam].first_packet
            if len(first_packet) > 0:
                if isinstance(self.objects[whoiam], RobotObject):
                    if self.objects[whoiam].receive_first(first_packet) is not None:
                        status = False
                elif isinstance(self.objects[whoiam], RobotObjectCollection):
                    if self.objects[whoiam].receive_first(whoiam, first_packet) is not None:
                        status = False

                if not status:
                    self._debug_print("Closing all from _send_first_packets")
                    self._close_ports("exit")
                    raise self._handle_error(
                        RobotObjectReceiveError(
                            "receive_first signalled to exit. whoiam ID: '%s'" % whoiam, first_packet),
                        traceback.format_stack()
                    )

                self.logger.record(None, whoiam, first_packet, "object")

    # ----- port management -----

    def _print_port_info(self, port):
        """
        Print the crashed port's info
        :param port:
        :return: None
        """
        if self.debug_enabled:
            self._debug_print(pprint.pformat(port.port_info.__dict__) + "\n")
            time.sleep(0.01)  # wait for error messages to print

    def _open_ports(self):
        """
        Start all port processes. Send start flag
        :return: None
        """
        # send start first
        for robot_port in self.ports.values():
            if not robot_port.send_start():
                self._close_ports("error")
                raise self._handle_error(
                    RobotSerialPortWritePacketError("Unable to send start packet!", self.prev_packet_info, robot_port),
                    traceback.format_stack()
                )

        # then start receiving packets
        for robot_port in self.ports.values():
            robot_port.start()

    def _stop_all_ports(self):
        """
        Close all robot port processes
        :return: None
        """
        self._debug_print("Closing all ports")

        for robot_port in self.ports.values():
            self._debug_print("closing", robot_port.whoiam)
            robot_port.stop()

        for port in self.ports.values():
            has_exited = port.has_exited()
            self._debug_print("%s, '%s' has %s" % (port.address, port.whoiam,
                                                   "exited" if has_exited else "not exited!!"))
            if not has_exited:
                raise self._handle_error(RobotSerialPortFailedToStopError(
                    "Port signalled error while stopping", self.prev_packet_info,
                    port), traceback.format_stack())

    def _close_ports(self, reason):
        """
        Kill all RobotSerialPort processes and close their serial ports
        :return: None
        """
        try:
            self._debug_print("Calling user's close function")
            self.close(reason)
        except BaseException as error:  # in case close contains an error
            self._stop_all_ports()
            raise self._handle_error(
                CloseSignalledExitError(error),
                traceback.format_stack()
            )

        self._send_commands()
        self._debug_print("sent last commands")
        self._stop_all_ports()

    # ----- event handling -----

    def _deliver_packet(self, dt, whoiam, packet):
        try:
            if isinstance(self.objects[whoiam], RobotObject):
                if self.objects[whoiam].receive(dt, packet) is not None:
                    self._debug_print(
                        "receive for object signalled to exit. whoiam ID: '%s', packet: %s" % (whoiam, repr(packet)))
                    return "exit"
            elif isinstance(self.objects[whoiam], RobotObjectCollection):
                if self.objects[whoiam].receive(dt, whoiam, packet) is not None:
                    self._debug_print("receive for collection signalled to exit. whoiam ID: '%s', packet: %s" % (
                        whoiam, repr(packet)))
                    return "exit"

        except BaseException as error:
            self._debug_print("Closing all from _deliver_packet")
            self._close_ports("error")
            raise self._handle_error(
                RobotObjectReceiveError(whoiam, packet),
                traceback.format_stack()
            )

    def _received(self, dt, whoiam, packet):
        try:
            if self.received(dt, whoiam, packet, "object") is False:
                self._debug_print(
                    "received signalled to exit. whoiam ID: '%s', packet: %s" % (whoiam, repr(packet)))
                return "exit"

            if self.current_whoiam in self._linked_functions:
                self._linked_functions[self.current_whoiam](self.dt(), self.current_packet, "object")

        except BaseException as error:
            self._debug_print("Closing all from _received")
            self._close_ports("error")
            raise self._handle_error(
                PacketReceivedError(error),
                traceback.format_stack()
            )

    def _send_commands(self):
        """
        Check every robot object. Send all commands if there are any
        :return:
        """
        for whoiam in self.objects.keys():
            robot_object = self.objects[whoiam]
            if isinstance(robot_object, RobotObject):
                command_packets = robot_object.command_packets
            elif isinstance(robot_object, RobotObjectCollection):
                command_packets = robot_object.command_packets[whoiam]
            else:
                break
            while not command_packets.empty():
                command = self.objects[whoiam].command_packets.get()
                self.logger.record(self.dt(), whoiam, command, "command")

                if not self.ports[whoiam].write_packet(command):
                    self._debug_print("Closing all from _send_commands")
                    self._close_ports("error")
                    raise self._handle_error(
                        RobotSerialPortWritePacketError(
                            "Failed to send command %s to '%s'" % (command, whoiam), self.prev_packet_info,
                            self.ports[whoiam]),
                        traceback.format_stack()
                    )

    def _record_debug_prints(self, dt, port):
        with port.print_out_lock:
            while not port.debug_print_outs.empty():
                self.logger.record(dt, port.whoiam, port.debug_print_outs.get(), "debug")

    def _handle_error(self, error, traceback):
        if self.logger.is_open:
            for port in self.ports.values():
                self._record_debug_prints(self.dt(), port)

            error_message = "".join(traceback[:-1])
            error_message += "%s: %s" % (error.__class__.__name__, error.args[0])
            error_message += "\n".join(error.args[1:])
            self.logger.record(self.dt(), error.__class__.__name__, error_message, "error")

        self._close_log()
        return error

    def _close_log(self):
        if self.logger.is_open():
            self._debug_print("Logger closing. Writing file to", self.logger.full_path)
            self.logger.close()

    def _debug_print(self, *strings, ignore_flag=False):
        if self.debug_enabled or ignore_flag:
            string = "[Interface] %s" % (" ".join([str(x) for x in strings]))
            print(string)
            self.logger.record(self.dt(), "Interface", string, "debug")
