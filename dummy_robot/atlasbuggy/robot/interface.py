import pprint
import time
from multiprocessing import Lock, Queue

import serial
import serial.tools.list_ports

from atlasbuggy.logs.logger import Logger
from atlasbuggy.robot.clock import Clock
from atlasbuggy.robot.errors import *
from atlasbuggy.robot.robotport import RobotSerialPort


class RobotInterface:
    def __init__(self, *robot_objects, joystick=None,
                 log_data=True, log_name=None, log_dir=None,
                 debug_prints=False, loop_updates_per_second=120, port_updates_per_second=1000):
        """
        :param robot_objects: subclasses of RobotObject
        :param joystick: A subclass instance of BuggyJoystick
        :param log_data: A boolean indicating whether or not the received data should be written to a log file
        :param log_name: The name of the log file. If None, it will be today's date and time
        :param log_dir: The directory of the log file. If None, it will be today's date.
            See project.py for details
        :param debug_prints: Enable verbose prints
        :param updates_per_second: How quickly each port process should run.
        """

        self.debug_prints = debug_prints
        self.loop_updates_per_second = loop_updates_per_second
        self.port_updates_per_second = port_updates_per_second
        self.lag_warning_thrown = False  # prevents the terminal from being spammed

        if log_dir is None:
            # if no directory provided, use the ":today" flag
            # (follows the project.py directory reference convention)
            log_dir = ":today"
        self.logger = Logger(log_name, log_dir)
        self.user_log_time0 = 0
        if log_data:
            self.logger.open()

        self.joystick = joystick

        self.clock = Clock(self.loop_updates_per_second)

        # a pipe from all port processes to the main loop
        self.packet_queue = Queue()
        self.queue_lock = Lock()

        self.objects = {}
        for robot_object in robot_objects:
            self.objects[robot_object.whoiam] = robot_object

        # open all available ports
        self.ports = {}
        for port_info in serial.tools.list_ports.comports():
            self._configure_port(port_info, self.port_updates_per_second)

        self._check_objects()
        self._check_ports()
        self._send_first_packets()

    def run(self):
        """
        Call this method to start the robot and to start receiving data

        This is where all threads start and stop. Stop events can be thrown by:
            loop returning False (live plot in the loop returned False for example)
            joystick returned False, the pygame window signalled to quit
            are_threads_running() returned False

        are_threads_running can be False for the following reasons:
            robot_port.is_running found that the serial thread has been hanging
                for more than 2 seconds
            robot_port.is_running found that should_stop is False

        should_stop can be False because:
            A packet wasn't parsed correctly
            The port never signalled ready
            There weren't enough ports to match the number of objects
            No ports with the requested whoiam ID were found
            The port wasn't matched to a RobotObject (it was unused)
        :return:
        """
        self._start_all()

        try:
            self.start()

            self.clock.start()
            self.user_log_time0 = time.time()

            while self._are_ports_active():
                if not self._dequeue_packets():
                    break

                if not self._main_loop():
                    break

                self._send_commands()

                if self.joystick is not None:
                    if self.joystick.update() is False:
                        break

                self.clock.update()
                if not self.lag_warning_thrown and self.dt > 0.5 and not self.clock.on_time:
                    print("Warning. Main loop is running slow.")
                    self.lag_warning_thrown = True

        except KeyboardInterrupt:
            pass

        self._close_all()

    @property
    def dt(self):
        return time.time() - self.user_log_time0

    def record(self, tag, string):
        self.logger.record(self.dt, tag, string)

    def packet_received(self, timestamp, whoiam, packet):
        """
        Overwrite this method

        A callback function for every time a packet is received

        :param timestamp: The time the packet arrived
        :param whoiam: Which robot object the packet went to
        :param packet: The packet received
        :return: return False if the program should exit for some reason
        """
        return True

    def loop(self):
        """
        Overwrite this method

        Similar to Arduino's loop function. This will be run in a while True loop

        :return: True if everything is ok. False if something signalled to close
        """
        return True

    def start(self):
        """
        Code that should be run just after the ports are opened

        :return: None
        """
        pass

    def close(self):
        """
        Code that should be run before the ports close

        :return: None
        """
        pass

    def _configure_port(self, port_info, updates_per_second):
        """
        Initialize a serial port recognized by pyserial.
        Only devices that are plugged in should be recognized

        :param port_info: an instance of serial.tools.list_ports_common.ListPortInfo
        :param updates_per_second: how often the port should update
        :return: None
        """
        if port_info.serial_number is not None:
            port = RobotSerialPort(port_info, self.debug_prints,
                                   self.packet_queue, self.queue_lock, updates_per_second)
            self.ports[port.whoiam] = port

            if not port.configured:
                self._close_all()
                self._print_port_info(port)

                raise RobotSerialPortNotConfiguredError("Port not configured!", port)

    def _check_objects(self):
        """
        Validate that all objects are assigned to ports. Throw RobotObjectNotFoundError otherwise
        :return: None
        """
        for whoiam in self.objects.keys():
            if whoiam not in self.ports.keys():
                self._close_all()
                self._print_port_info(self.ports[whoiam])

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

                RobotSerialPortUnassignedError("Port not assigned to an object.", self.ports[whoiam])

    def _send_first_packets(self):
        """
        Send each port's first packet to the corresponding object if it isn't an empty string
        :return: None
        """
        for whoiam in self.objects.keys():
            first_packet = self.ports[whoiam].first_packet
            if len(first_packet) > 0:
                self.objects[whoiam].receive_first(first_packet)

                self.logger.record(-1, whoiam, first_packet)

    def _print_port_info(self, port):
        """
        Print the crashed port's info
        :param port:
        :return:
        """
        if self.debug_prints:
            pprint.pprint(port.port_info.__dict__)
            print()
            time.sleep(0.01)

    def _start_all(self):
        """
        Start all port processes
        :return: None
        """
        for whoiam in self.ports.keys():
            self.ports[whoiam].start()

    def _stop_all_ports(self):
        if self.debug_prints:
            print("Closing all ports")
        for robot_port in self.ports.values():
            robot_port.stop()

    def _close_all(self):
        """
        Kill all RobotSerialPort threads and close their serial ports
        :return: None
        """
        try:
            self.close()
        except BaseException as error:
            self._stop_all_ports()
            raise CloseSignalledExitError(error)

        self._stop_all_ports()

    def _are_ports_active(self):
        """
        Using each robot port's is_running method, check if the processes are running properly
        :return:
        """
        for robot_port in self.ports.values():
            status = robot_port.is_running()
            if status < 1:
                self._close_all()
                if status == 0:
                    raise RobotSerialPortSignalledExitError("Port with ID '%s' signalled to exit" % robot_port.whoiam)
                elif status == -1:
                    raise RobotSerialPortTimeoutError("Port with ID '%s' timed out" % robot_port.whoiam)
        return True

    def _dequeue_packets(self):
        """
        dequeue all packets from packet_queue. Pass them to the corresponding robot objects
        :return: what packet_received returns (True or False signalled to exit or not)
        """
        with self.queue_lock:
            while not self.packet_queue.empty():
                whoiam, timestamp, packet = self.packet_queue.get()
                self.objects[whoiam].receive(packet)

                self.logger.record(timestamp, whoiam, packet)
                try:
                    if self.packet_received(timestamp, whoiam, packet) is False:
                        if self.debug_prints:
                            print("packet_received signalled to exit")
                        return False
                except BaseException as error:
                    self._close_all()
                    raise PacketReceivedSignalledExitError(error)
            return True

    def _main_loop(self):
        """
        Call the loop method safely
        :return: True or False signalled to exit or not
        """
        try:
            if self.loop() is False:
                if self.debug_prints:
                    print("loop signalled to exit")
                return False
        except BaseException as error:
            self._close_all()
            raise LoopSignalledError(error)

        return True

    def _send_commands(self):
        """
        Check every robot object. Send all commands if there are any
        :return:
        """
        for whoiam in self.objects.keys():
            while not self.objects[whoiam].command_packets.empty():
                command = self.objects[whoiam].command_packets.get()
                if not self.ports[whoiam].write_packet(command):
                    self._close_all()
                    raise RobotSerialPortWritePacketError("Failed to send command:", command)
