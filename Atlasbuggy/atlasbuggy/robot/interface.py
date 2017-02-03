"""
This class manages all robot ports and pass data received to the corresponding robot objects.
"""

import pprint
import time
from multiprocessing import Lock, Queue, Value
import threading
import traceback

import serial
import serial.tools.list_ports

from atlasbuggy.logfiles.logger import Logger
from atlasbuggy.robot.clock import Clock
from atlasbuggy.robot.errors import *
from atlasbuggy.robot.robotport import RobotSerialPort
from atlasbuggy.robot.robotobject import RobotObject
from atlasbuggy.robot.robotcollection import RobotObjectCollection


class RobotInterface:
    def __init__(self, *robot_objects, joystick=None,
                 log_data=True, log_name=None, log_dir=None,
                 debug_prints=False, debug_to_log=False, loop_updates_per_second=120, port_updates_per_second=1000):
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

        self.debug_to_log = debug_to_log  # TODO: put debug messages into a log file
        if debug_to_log:
            self.debug_prints = True
        else:
            self.debug_prints = debug_prints

        self.loop_ups = loop_updates_per_second
        self.port_ups = port_updates_per_second
        self.lag_warning_thrown = False  # prevents the terminal from being spammed
        self.prev_whoiam = ""
        self.prev_packet_info = [None, None, None]

        if log_dir is None:
            # if no directory provided, use the ":today" flag
            # (follows the project.py directory reference convention)
            log_dir = ":today"
        self.logger = Logger(log_name, log_dir)
        self.start_time = 0
        if log_data:
            self.logger.open()

        self.joystick = joystick

        self.clock = Clock(self.loop_ups)

        # a pipe from all port processes to the main loop
        self.packet_queue = Queue()
        self.packet_counter = Value('i', 0)
        self.port_lock = Lock()

        # assign robot objects by whoiam ID
        self.objects = {}
        self.inactive_ids = set()
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

        # open all available ports using multithreading
        self.ports = {}
        threads = []
        for port_info in serial.tools.list_ports.comports():
            config_thread = threading.Thread(target=self._configure_port, args=(port_info, self.port_ups))
            threads.append(config_thread)
            config_thread.start()

        for thread in threads:
            thread.join()

        for port in self.ports.values():
            if not port.configured:
                self._print_port_info(port)
                raise RobotSerialPortNotConfiguredError("Port not configured!", self.prev_packet_info, port)

        self._check_objects()  # check that all objects are assigned a port
        self._check_ports()  # check that all ports are assigned an object
        self._send_first_packets()  # distribute initialization packets

        for whoiam in self.ports.keys():
            self._debug_print("[%s] has ID '%s'" % (self.ports[whoiam].address, whoiam))

        self._debug_print("Ignored IDs:")
        for whoiam in self.inactive_ids:
            self._debug_print(whoiam)

    def run(self):
        """
        Call this method to start the robot and to start receiving data

        This is where all processes start and stop. Stop events can be thrown by:
            loop returning False (live plot in the loop returned False for example),
            joystick returned False (the pygame window signalled to quit),
            _are_ports_active() returned False

        _are_ports_active can be False for the following reasons:
            robot_port.is_running found that the process has been hanging
                for more than 2 seconds
            robot_port.is_running found that should_stop is False indicating the process crashed

        should_stop can be False because:
            A packet wasn't parsed correctly
            The port never signalled ready
            There weren't enough ports to match the number of objects
            No ports with the requested whoiam ID were found
            The port wasn't matched to a RobotObject (it was unused)
        :return: None
        """

        self.start_time = time.time()
        self.clock.start(self.start_time)

        self._start_all()
        try:
            self.start()  # call user's start method (empty by default)
        except BaseException as error:
            self._debug_print("Closing all from user's start")
            self._close_all()
            raise StartSignalledError("Overridden start method threw an exception")

        try:
            while self._are_ports_active():
                if not self._dequeue_packets():  # calls packet_received if the queue is occupied
                    break

                if not self._main_loop():  # calls loop no matter what
                    break

                self._send_commands()  # sends all commands in each robot object's command queue

                if self.joystick is not None:
                    if self.joystick.update() is False:
                        break

                self.clock.update()  # maintain a constant loop speed
                if not self.lag_warning_thrown and self.dt > 0.1 and not self.clock.on_time:
                    print("Warning. Main loop is running slow.")
                    self.lag_warning_thrown = True

        except KeyboardInterrupt:
            pass

        self._debug_print("Closing all from run")
        self._close_all()

    # ----- utility methods -----

    @property
    def dt(self):
        """
        Access the clock. This time uses the same reference as all other objects
        :return: time since the program's start in seconds
        """
        return time.time() - self.start_time

    def record(self, tag, string):
        """
        Record data not created by a robot object.

        :param tag: Unique tag similar to whoiam ID. Make sure these don't overlap with any sensors
        :param string: Similar to a packet. String data to record
        :return: None
        """
        self.logger.record(self.dt, tag, string, packet_type="user")

    def queue_len(self):
        return self.packet_counter.value

    def did_receive(self, arg):
        if isinstance(arg, RobotObject):
            return arg.whoiam == self.prev_whoiam
        elif isinstance(arg, RobotObjectCollection):
            for whoiam in arg.whoiam_ids:
                return whoiam == self.prev_whoiam
        else:
            return arg == self.prev_whoiam

    # ----- overridable methods -----

    def packet_received(self, timestamp, whoiam, packet):
        """
        Override this method

        A callback function for every time a packet is received

        :param timestamp: The time the packet arrived
        :param whoiam: Which robot object the packet went to
        :param packet: The packet received
        :return: return False if the program should exit for some reason, True or None otherwise
        """

    def loop(self):
        """
        Override this method

        Similar to Arduino's loop function. This will be run in a while True loop

        :return: True or None if everything is ok. False if something signalled to close
        """
        pass

    def start(self):
        """
        Code that should be run just after the ports are opened and the clock is started

        :return: None
        """
        pass

    def close(self):
        """
        Code that should be run before the ports close

        :return: None
        """
        pass

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
            port = RobotSerialPort(port_info, self.debug_prints,
                                   self.packet_queue, self.port_lock, self.packet_counter, updates_per_second)
            if port.whoiam in self.ports.keys():
                self._close_all()
                self._print_port_info(port)
                raise RobotSerialPortWhoiamIdTaken("whoiam ID already being used by another port!",
                                                   self.prev_packet_info, port)
            elif port.whoiam in self.inactive_ids:
                port.stop()
            else:
                self.ports[port.whoiam] = port

    def _check_objects(self):
        """
        Validate that all objects are assigned to ports. Throw RobotObjectNotFoundError otherwise
        :return: None
        """
        for whoiam in self.objects.keys():
            if whoiam not in self.ports.keys():
                self._close_all()
                raise RobotObjectNotFoundError("Failed to assign robot object with ID '%s'" % whoiam)

    def _check_ports(self):
        """
        Validate that all ports are assigned to objects. Throw RobotSerialPortUnassignedError otherwise
        :return: None
        """
        for whoiam in self.ports.keys():
            if whoiam not in self.objects.keys():
                self._close_all()
                self._print_port_info(self.ports[whoiam])

                raise RobotSerialPortUnassignedError("Port not assigned to an object.", self.prev_packet_info,
                                                     self.ports[whoiam])

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
                    if self.objects[whoiam].receive_first(first_packet) is False:
                        status = False
                elif isinstance(self.objects[whoiam], RobotObjectCollection):
                    if self.objects[whoiam].receive_first(whoiam, first_packet) is False:
                        status = False

                if not status:
                    self._debug_print("Closing all from _send_first_packets")
                    self._close_all()
                    raise RobotObjectReceiveError(
                        "receive_first signalled to exit. whoiam ID: '%s'" % whoiam, first_packet)

                self.logger.record(-1, whoiam, first_packet, packet_type="object")

    # ----- port management -----

    def _print_port_info(self, port):
        """
        Print the crashed port's info
        :param port:
        :return: None
        """
        if self.debug_prints:
            self._debug_print(pprint.pformat(port.port_info.__dict__) + "\n")
            time.sleep(0.01)  # wait for error messages to print

    def _start_all(self):
        """
        Start all port processes. Send start flag
        :return: None
        """
        # send start first
        for robot_port in self.ports.values():
            if not robot_port.send_start():
                self._close_all()
                raise RobotSerialPortWritePacketError("Unable to send start packet!", self.prev_packet_info, robot_port)

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
            robot_port.stop()
            robot_port.wait_for_close()

    def _close_all(self):
        """
        Kill all RobotSerialPort processes and close their serial ports
        :return: None
        """
        try:
            self._debug_print("Calling user's close function")
            self.close()  # call the user's close function
        except BaseException as error:  # in case close contains an error
            self._stop_all_ports()
            self.logger.close()
            raise CloseSignalledExitError(error)

        self._stop_all_ports()
        self.logger.close()

    def _are_ports_active(self):
        """
        Using each robot port's is_running method, check if the processes are running properly
        An error will be thrown if not.

        :return: True if the ports are ok
        """
        for robot_port in self.ports.values():
            status = robot_port.is_running()
            if status < 1:
                self._debug_print("Closing all from _are_ports_active")
                self._close_all()
                if status == 0:
                    raise RobotSerialPortNotConfiguredError(
                        "Port with ID '%s' isn't configured!" % robot_port.whoiam, self.prev_packet_info, robot_port)
                elif status == -1:
                    raise RobotSerialPortSignalledExitError(
                        "Port with ID '%s' signalled to exit" % robot_port.whoiam, self.prev_packet_info, robot_port)
        return True

    # ----- event handling -----

    def _deliver_packet(self, dt, whoiam, packet):
        try:
            if isinstance(self.objects[whoiam], RobotObject):
                if self.objects[whoiam].receive(dt, packet) is False:
                    self._debug_print(
                        "receive for object signalled to exit. whoiam ID: '%s', packet: %s" % (whoiam, repr(packet)))
                    return False
            elif isinstance(self.objects[whoiam], RobotObjectCollection):
                if self.objects[whoiam].receive(dt, whoiam, packet) is False:
                    self._debug_print("receive for collection signalled to exit. whoiam ID: '%s', packet: %s" % (
                        whoiam, repr(packet)))
                    return False
        except BaseException as error:
            self._debug_print("Closing all from _deliver_packet")
            self._close_all()
            raise RobotObjectReceiveError(whoiam, packet)

    def _signal_received(self, dt, whoiam, packet):
        try:
            self.prev_whoiam = whoiam
            if self.packet_received(dt, whoiam, packet) is False:
                self._debug_print(
                    "packet_received signalled to exit. whoiam ID: '%s', packet: %s" % (whoiam, repr(packet)))
                return False
        except BaseException as error:
            self._debug_print("Closing all from _signal_received")
            self._close_all()
            raise PacketReceivedError(error)
        return True

    def _dequeue_packets(self):
        """
        dequeue all packets from packet_queue. Pass them to the corresponding robot objects
        :return: what packet_received returns (True or False signalled to exit or not)
        """
        with self.port_lock:
            while not self.packet_queue.empty():
                whoiam, timestamp, packet = self.packet_queue.get()
                self.packet_counter.value -= 1
                dt = timestamp - self.start_time

                self.prev_packet_info[0] = whoiam
                self.prev_packet_info[1] = dt
                self.prev_packet_info[2] = packet

                if self._deliver_packet(dt, whoiam, packet) is False:
                    return False
                self.logger.record(dt, whoiam, packet, packet_type="object")
                if not self._signal_received(dt, whoiam, packet):
                    return False

        return True

    def _main_loop(self):
        """
        Call the loop method safely
        :return: True or False signalled to exit or not
        """
        try:
            if self.joystick is not None:
                self.joystick.update()
            if self.loop() is False:
                self._debug_print("loop signalled to exit")
                return False
        except BaseException as error:
            self._debug_print("Closing all from _main_loop")
            self._close_all()
            raise LoopSignalledError(error)

        return True

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
                self.logger.record(self.dt, whoiam, command, packet_type="command")

                if not self.ports[whoiam].write_packet(command):
                    self._debug_print("Closing all from _send_commands")
                    self._close_all()
                    raise RobotSerialPortWritePacketError(
                        "Failed to send command %s to '%s'" % (command, whoiam), self.prev_packet_info,
                        self.ports[whoiam])

    # def _handle_error(self, traceback):

    def _debug_print(self, *strings, ignore_flag=False):
        if self.debug_prints or ignore_flag:
            string = "".join(strings)
            print("[Interface] %s" % string)
