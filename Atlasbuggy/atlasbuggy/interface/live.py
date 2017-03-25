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
from atlasbuggy.interface.base import BaseInterface


class RobotRunner(BaseInterface):
    loop_updates_per_second = 120  # How quickly the main thread should run
    port_updates_per_second = 1000  # How quickly each port process should run

    def __init__(self, robot, joystick=None, log_data=True, log_name=None, log_dir=None, debug_prints=False):
        """
        :param robot: a subclass instance of the atlasbuggy.robot.Robot class
        :param joystick: a subclass instance of the atlasbuggy.buggyjoystick.BuggyJoystick class
        :param log_data: enable data logging (debug prints logged even if they are not enabled)
        :param log_name: log file name (today's date is substituted if None (YYYY_MM_DD))
        :param log_dir: log directory (today's date is substituted if None (YYYY_MM_DD))
            Tuple feature: ("some_data", None) -> produces "some_data/YYYY_MM_DD"
        :param debug_prints: print debug messages
        """

        super(RobotRunner, self).__init__(robot, debug_prints, "Interface")

        # clock properties
        self.loop_ups = RobotRunner.loop_updates_per_second
        self.port_ups = RobotRunner.port_updates_per_second
        self.lag_warning_thrown = False  # prevents the terminal from being spammed
        self.clock = Clock(self.loop_ups)
        self.start_time = 0

        # keep the last packet info for crash info
        self.prev_packet_info = [None, None, None]

        # pass around reference to joystick
        self.joystick = joystick
        self.robot.joystick = joystick

        # initialize logger
        self.logger = Logger(log_name, log_dir)
        self.robot.logger = self.logger
        if log_data:
            self.logger.open()
            print("Writing to:", self.logger.full_path)

        for robot_object in self.robot.objects.values():
            robot_object.is_live = True
        self.robot.is_live = True
        self.robot.debug_enabled = self.debug_enabled

        # create a pipe from all port processes to the main loop
        self.packet_queue = Queue()
        self.packet_counter = Value('i', 0)
        self.port_lock = Lock()

        # open all available ports
        self.ports = {}
        threads = []

        # call _configure_port for each available serial port
        self.duplicate_id_error = [False, None]  # [did error occur, whoiam ID]
        for port_info in serial.tools.list_ports.comports():
            config_thread = threading.Thread(target=self._configure_port, args=(port_info, self.port_ups))
            threads.append(config_thread)
            config_thread.start()

        # wait for threads to finish
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

        # throw an error if a port isn't configured correctly
        for port_name, port in self.ports.items():
            if not port.configured:
                self._print_port_info(port)
                raise self._handle_error(
                    RobotSerialPortNotConfiguredError("Port not configured!", self.prev_packet_info, port),
                    traceback.format_stack()
                )

        self._debug_print("Discovered ports:", list(self.ports.keys()))

        self._check_objects()  # check that all objects are assigned a port
        self._check_ports()  # check that all ports are assigned an object

        for whoiam in self.ports.keys():
            self._debug_print("[%s] has ID '%s'" % (self.ports[whoiam].address, whoiam))

        # notify user of disabled robot objects
        if len(self.robot.inactive_ids) > 0 and self.debug_enabled:
            self._debug_print("Ignored IDs:")
            for whoiam in self.robot.inactive_ids:
                self._debug_print(whoiam)

        self._send_first_packets()  # distribute initialization packets

    def _start(self):
        """
        Start all robot ports
        """
        self.start_time = time.time()
        self.clock.start(self.start_time)

        self._open_ports()

        self._debug_print("Interface is starting")

        try:
            return self.robot.start()  # call user's start method (empty by default)
        except BaseException as error:
            self._debug_print("Closing all from user's start")
            self._close_ports("error")
            raise self._handle_error(
                StartSignalledError("User's start method threw an exception"),
                traceback.format_stack()
            )

    def _loop(self):
        """
        Loop event. Update joystick (if not None) and call user's loop method
        """
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
            status = self.robot.loop()
            if status is not None:
                self._debug_print("User's loop method signalled to exit")
                return status
        except BaseException as error:
            self._debug_print("Closing all from user's loop")
            self._close_ports("error")
            self._debug_print("_loop signalled an error")
            raise self._handle_error(
                LoopSignalledError(error),
                traceback.format_stack()
            )

        self._send_commands()  # sends all commands in each robot object's command queue

        self.clock.update()  # maintain a constant loop speed
        if not self.lag_warning_thrown and self.robot.current_timestamp is not None and \
                        self.robot.current_timestamp > 0.1 and not self.clock.on_time:
            self._debug_print("Warning. Main loop is running slow. Clock is behind by %ss" % abs(self.clock.offset))
            self.lag_warning_thrown = True

    def _update(self):
        """
        dequeue all packets from packet_queue. Pass them to the corresponding robot objects
        :return: what packet_received returns (None or string signalled to exit or not)
        """

        # gain access to port shared properties
        with self.port_lock:
            while not self.packet_queue.empty():

                # dequeue packet info
                self.robot.current_whoiam, timestamp, self.robot.current_packet = self.packet_queue.get()
                self.packet_counter.value -= 1
                self.robot.current_timestamp = timestamp - self.start_time

                self.robot.queue_len = self.packet_counter.value

                # deliver packet to correct robot object
                status = self._deliver_packet(self.robot.current_timestamp, self.robot.current_whoiam,
                                              self.robot.current_packet)
                if status is not None:
                    return status

                # record packet and debug prints from the port
                if self.logger.is_open():
                    self.logger.record(self.robot.current_timestamp, self.robot.current_whoiam,
                                       self.robot.current_packet, "object")
                    self._record_debug_prints(self.robot.current_timestamp, self.ports[self.robot.current_whoiam])

                # call robot's receive method
                status = self._received(self.robot.current_timestamp, self.robot.current_whoiam,
                                        self.robot.current_packet)
                if status is not None:
                    return status

        # if no packets have been received for a while, update the timestamp with the current clock time
        current_real_time = time.time() - self.start_time
        if self.robot.current_timestamp is None or current_real_time - self.robot.current_timestamp > 0.01:
            self.robot.current_timestamp = current_real_time

    def _close(self, reason):
        if not self.close_called:
            self._debug_print("Closing all")

            self._close_log()
            self._close_ports(reason)

            self.close_called = True

    def _should_run(self):
        """
        Check if the processes are running properly. An error will be thrown if not.

        :return: True if the ports are ok
        """

        for robot_port in self.ports.values():
            status = robot_port.is_running()
            if status < 1:
                self._debug_print("Closing all from _are_ports_active")
                self._close_ports("error")
                self._debug_print("status:", status)
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

    # ----- configuration methods -----

    def _configure_port(self, port_info, updates_per_second):
        """
        Initialize a serial port recognized by pyserial.
        Only devices that are plugged in should be recognized

        :param port_info: an instance of serial.tools.list_ports_common.ListPortInfo
        :param updates_per_second: how often the port should update
        """
        if port_info.vid is not None:
            # instantiate RobotSerialPort
            port = RobotSerialPort(port_info, self.debug_enabled,
                                   self.packet_queue, self.port_lock, self.packet_counter, updates_per_second)
            self._debug_print("whoiam", port.whoiam)

            # check for duplicate IDs
            if port.whoiam in self.ports.keys():
                self.duplicate_id_error[0] = True
                self.duplicate_id_error[1] = port

            # check if port abides protocol. Warn the user and stop the port if not (ignore it essentially)
            elif port.configured and (not port.abides_protocols or port.whoiam is None):
                self._debug_print("Warning! Port '%s' does not abide Atlasbuggy protocol!" % port.address)
                port.stop()

            # check if port is configured correctly. If not don't add it to list of ports (ignore it)
            elif not port.configured:
                self._debug_print("Port not configured! '%s'" % port.address)

            # disable ports if the corresponding object if disabled
            elif port.whoiam in self.robot.inactive_ids:
                port.stop()

            # add the port if configured and abides protocol
            else:
                self.ports[port.whoiam] = port

    def _check_objects(self):
        """
        Validate that all enabled objects are assigned to ports. Throw RobotObjectNotFoundError otherwise
        """
        for whoiam in self.robot.objects.keys():
            if whoiam not in self.ports.keys():
                self._close_ports("error")
                raise self._handle_error(
                    RobotObjectNotFoundError("Failed to assign robot object with ID '%s'" % whoiam),
                    traceback.format_stack()
                )

    def _check_ports(self):
        """
        Validate that all ports are assigned to enabled objects. Warn the user otherwise
            (this allows for ports not listed in objects to be plugged in but not used)
        """
        used_ports = {}
        for whoiam in self.ports.keys():
            if whoiam not in self.robot.objects.keys():
                self._debug_print("Warning! Port ['%s', %s] is unused!" %
                                  (self.ports[whoiam].address, whoiam), ignore_flag=True)
            else:
                # only append port if its used. Ignore it otherwise
                used_ports[whoiam] = self.ports[whoiam]

                # if a robot object signals it wants a different baud rate, change to that rate
                object_baud = self.robot.objects[whoiam].baud
                if object_baud is not None and object_baud != self.ports[whoiam].baud_rate:
                    self.ports[whoiam].change_rate(object_baud)
        self.ports = used_ports

    def _send_first_packets(self):
        """
        Send each port's first packet to the corresponding object if it isn't an empty string
        """
        status = True
        for whoiam in self.robot.objects.keys():
            first_packet = self.ports[whoiam].first_packet
            if len(first_packet) > 0:
                # different cases for objects and object collections, pass to receive_first
                if isinstance(self.robot.objects[whoiam], RobotObject):
                    if self.robot.objects[whoiam].receive_first(first_packet) is not None:
                        status = False
                elif isinstance(self.robot.objects[whoiam], RobotObjectCollection):
                    if self.robot.objects[whoiam].receive_first(whoiam, first_packet) is not None:
                        status = False

                if not status:
                    self._debug_print("Closing all from _send_first_packets")
                    self._close_ports("exit")
                    raise self._handle_error(
                        RobotObjectReceiveError(
                            "receive_first signalled to exit. whoiam ID: '%s'" % whoiam, first_packet),
                        traceback.format_stack()
                    )

                # record first packets
                self.logger.record(None, whoiam, first_packet, "object")

    # ----- port management -----

    def _print_port_info(self, port):
        """
        Print the crashed port's info
        :param port:
        """
        if self.debug_enabled:
            self._debug_print(pprint.pformat(port.port_info.__dict__) + "\n")
            time.sleep(0.01)  # wait for error messages to print

    def _open_ports(self):
        """
        Start all port processes. Send start flag
        """
        for robot_port in self.ports.values():
            if not robot_port.send_start():
                self._close_ports("error")
                raise self._handle_error(
                    RobotSerialPortWritePacketError("Unable to send start packet!", self.prev_packet_info, robot_port),
                    traceback.format_stack()
                )

        # start port processes
        for robot_port in self.ports.values():
            robot_port.start()

    def _stop_all_ports(self):
        """
        Close all robot port processes
        """
        self._debug_print("Closing all ports")

        # stop port processes
        for robot_port in self.ports.values():
            self._debug_print("closing", robot_port.whoiam)
            robot_port.stop()

        for robot_port in self.ports.values():
            self._debug_print("[%s] Port previous packets: read: %s, write %s" % (
                robot_port.whoiam,
                robot_port.prev_read_packets, repr(robot_port.prev_write_packet))
            )

        # check if the port exited properly
        for port in self.ports.values():
            has_exited = port.has_exited()
            self._debug_print("%s, '%s' has %s" % (port.address, port.whoiam,
                                                   "exited" if has_exited else "not exited!!"))
            if not has_exited:
                raise self._handle_error(RobotSerialPortFailedToStopError(
                    "Port signalled error while stopping", self.prev_packet_info,
                    port), traceback.format_stack())
        self._debug_print("All ports exited")

    def _close_ports(self, reason):
        """
        Close all RobotSerialPort processes and close their serial ports
        """
        if self.close_called:
            self._debug_print("_close_ports already called. Ignoring")
            return
        self.close_called = True

        try:
            self._debug_print("Calling user's close function")
            self.robot.close(reason)
        except BaseException as error:  # in case close contains an error
            self._stop_all_ports()
            raise self._handle_error(
                CloseSignalledExitError(error),
                traceback.format_stack()
            )

        self._send_commands()
        self._debug_print("Sent last commands")
        self._stop_all_ports()
        self._debug_print("Closed ports successfully")

    # ----- event handling -----

    def _deliver_packet(self, dt, whoiam, packet):
        """
        Distribute packet to robot objects's receive method. If the method throws an error, catch it and close all ports

        :param dt: current time relative to program start
        :param whoiam: destination object's whoiam ID
        :param packet: packet string
        """
        try:
            if isinstance(self.robot.objects[whoiam], RobotObject):
                if self.robot.objects[whoiam].receive(dt, packet) is not None:
                    self._debug_print(
                        "receive for object signalled to exit. whoiam ID: '%s', packet: %s" % (whoiam, repr(packet)))
                    return "exit"
            elif isinstance(self.robot.objects[whoiam], RobotObjectCollection):
                if self.robot.objects[whoiam].receive(dt, whoiam, packet) is not None:
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
        """
        Distribute packet to robot's receive method. If the method throws an error, catch it and close all ports

        Call any linked callback functions

        :param dt: current time relative to program start
        :param whoiam: destination object's whoiam ID
        :param packet: packet string
        """
        try:
            status = self.robot.received(dt, whoiam, packet, "object")
            if status is not None:
                self._debug_print(
                    "received signalled to exit. whoiam ID: '%s', packet: %s" % (whoiam, repr(packet)))
                return status

            if self.robot.current_whoiam in self.robot.linked_functions:
                return self.robot.linked_functions[self.robot.current_whoiam](
                    self.robot.current_timestamp, self.robot.current_packet, "object")

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
        """
        for whoiam in self.robot.objects.keys():

            # extract command_packets queue
            robot_object = self.robot.objects[whoiam]
            if isinstance(robot_object, RobotObject):
                command_packets = robot_object.command_packets
            elif isinstance(robot_object, RobotObjectCollection):
                command_packets = robot_object.command_packets[whoiam]
            else:
                break

            # loop through all commands and send them
            while not command_packets.empty():
                command = self.robot.objects[whoiam].command_packets.get()

                # log sent command
                self.logger.record(self.robot.current_timestamp, whoiam, command, "command")

                # if write packet fails, throw an error
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
        """
        Take all of the port's queued debug messages and record them
        :param dt: current timestamp
        :param port: RobotSerialPort
        """
        with port.print_out_lock:
            while not port.debug_print_outs.empty():
                self.logger.record(dt, port.whoiam, port.debug_print_outs.get(), "debug")

    def _handle_error(self, error, traceback):
        """
        Format the thrown error for logging. Return it after
        :param error: Error being thrown
        :param traceback: stack trace of error
        :return: Error being thrown
        """
        if self.logger.is_open:
            for port in self.ports.values():
                self._record_debug_prints(self.robot.current_timestamp, port)
            self._debug_print("Port debug prints recorded")

            error_message = "".join(traceback[:-1])
            error_message += "%s: %s" % (error.__class__.__name__, error.args[0])
            error_message += "\n".join(error.args[1:])
            self.logger.record(self.robot.current_timestamp, error.__class__.__name__, error_message, "error")

        self._close_log()
        self._debug_print("logger closed")
        return error

    def _close_log(self):
        if self.logger.is_open():
            self._debug_print("Logger closing. Writing file to", self.logger.full_path)
            self.logger.close()

    def _debug_print(self, *values, ignore_flag=False):
        string = "[%s] %s" % (self.debug_name, " ".join([str(x) for x in values]))
        self.logger.record(self.robot.current_timestamp, self.debug_name, string, "debug")

        if self.debug_enabled or ignore_flag:
            print(string)
