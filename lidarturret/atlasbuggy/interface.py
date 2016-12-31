import time
import traceback
from queue import Queue
from threading import Thread
import serial
import serial.tools.list_ports
from serial.serialutil import SerialException, portNotOpenError
import pprint

from atlasbuggy.logs import Logger
from atlasbuggy.errors import *


class RobotSerialPort(Thread):
    """
    A multi-threaded wrapper for an instance of pyserial Serial.
    A RobotInterface class manages all instances of this class.
    This class is for internal use only
    """

    def __init__(self, port_info, logger, log_data, debug_prints):
        """
        :param port_info: A ListPortInfo object returned by serial.tools.list_ports.comports()
        :param logger: An instance of the Logger class defined in logs.py
        :param log_data: A boolean that signals whether or not to log data (to use the logger instance or not)
        :param debug_prints: Enable verbose print statements
        """
        self.address = port_info.device  # directory to open (in /dev)
        self.port_info = port_info

        # status variables
        self.debug_prints = debug_prints
        self.configured = True
        self.error_message = None
        self.port_assigned = False
        self.stop_sent = False

        self.start_time = time.time()
        self.thread_time = 0

        self.whoiam = None  # ID tag of the microcontroller
        self.whoiam_header = "iam"  # whoiam packets start with "iam"
        self.packet_end = "\n"  # what this microcontroller's packets end with
        self.baud_rate = 115200

        # object to pass packets to for parsing, set externally after all
        # discovered ports have been opened
        self.robot_object = None

        # buffer for putting packets into
        self.buffer = ""
        self.first_packet = ""

        self.log_data = log_data
        self.logger = logger  # reference to instance of the Logger class

        # attempt to open the serial port
        try:
            self.serial_ref = serial.Serial(port=self.address, baudrate=self.baud_rate)
        except SerialException as error:
            self.handle_error(error)

        if self.configured:
            # check if the microcontroller signals ready
            if self.is_ready():
                # Find the ID of this port. The ports will be matched up to the correct RobotObject later
                self.find_whoiam()

        super(RobotSerialPort, self).__init__()

    # ----- initialization methods -----

    def is_ready(self):
        """
        Check if microcontroller is ready. This method will wait 1 second
        until the packet is received
        :return: None
        """

        is_ready = self.check_protocol("ready?", self.parse_for_ready_packet)

        if is_ready:
            if self.debug_prints:
                print("%s is ready" % self.address)
            else:
                print("%s is ready" % self.address, end=" ")
        else:
            print("%s is NOT ready!" % self.address)

        return is_ready

    @staticmethod
    def parse_for_ready_packet(packets):
        return "ready!" in packets

    def find_whoiam(self):
        """
        Get the whoiam packet from the microcontroller. This method will wait 1 second
        until the packet is received

        When the packet is found, parse_whoiam_packet is called and whoiam is assigned
        :return: whoiam packet and first_packet
        """

        abides_protocol = self.check_protocol("whoareyou", self.parse_for_whoiam_packet)

        # record the first packet if it exists
        if abides_protocol and self.log_data and len(self.first_packet) > 0:
            self.logger.record(self.whoiam, self.first_packet)

        if self.whoiam is not None:
            if self.debug_prints:
                print("%s has ID '%s'" % (self.address, self.whoiam))
            else:
                print("and has ID '%s'" % self.whoiam)

    def parse_for_whoiam_packet(self, packets):
        """
        Check if the packet is a whoiam packet (starts with "iam")
        Parses any accompanying data

        example: "iamlidar\t0,1,2"
            \t indicates extra data to be sent to robot_object.receive_first
            if there is no tab character, first_packet is "" and receive_first is not called
        :param packet: a packet that may or may not be a whoiam packet
        :return: whoiam and first_packet if they are found None and "" otherwise
        """
        for packet in packets:
            # if packet contains whoiam_header
            if packet[0:len(self.whoiam_header)] == self.whoiam_header:
                # remove the "iam" header
                packet = packet[len(self.whoiam_header):]

                # whoiam packets can contain initialization data, this is indicated by a tab character.
                # If it exists, assign the rest of the packet to first_packet
                data_index = packet.find("\t")
                if data_index != -1:
                    self.whoiam = packet[:data_index]
                    self.first_packet = packet[data_index + 1:]
                else:
                    self.whoiam = packet
                    self.first_packet = ""

                return True
        return False

    def check_protocol(self, send_packet, receive_fn):
        if self.debug_prints:
            print("Checking '%s' protocol" % send_packet)
        if not self.write_packet(send_packet):
            return False

        start_time = time.time()
        abides_protocol = False

        # wait for the correct response
        while not abides_protocol:
            packets, status = self.read_packets()
            if not status:
                self.handle_error("Serial read failed... Board never signaled ready")
                return False
            if (time.time() - start_time) > 0.1:
                self.handle_error("Didn't receive response for packet '%s'. Operation timed out." % send_packet)
                return False

            self.print_packets(packets)
            abides_protocol = receive_fn(packets)

        return True  # when the while loop exits, abides_protocol must be True

    def handle_error(self, error):
        self.configured = False
        if self.error_message is None:
            self.error_message = traceback.format_stack()
            self.error_message.append("%s: %s" % (error.__class__.__name__, str(error)))

    # ----- run methods -----

    def run(self):
        """Called when RobotSerialPort.start is called (inherited from threading.Thread)"""
        while not self.stop_sent:
            # update the internal timer. Acts as a check to see if the thread is running properly
            self.thread_time = int(time.time() - self.start_time)
            if not self.serial_ref.is_open:
                self.stop()
                raise RobotSerialPortClosedPrematurelyError("Serial port isn't open for some reason...")

            if self.serial_ref.in_waiting > 0:
                # read every possible character available and split them into packets
                packets, status = self.read_packets()
                if not status:
                    self.stop()
                    raise RobotSerialPortReadPacketError("Failed to read packets")

                self.parse_packets(packets)
                self.send_commands()

    def send_commands(self):
        # to avoid an endless loop of objects having references to
        # ports and ports to objects, the port just checks if the
        # object updated its command packet and sends it

        # dequeue all command packets and write them (send from the end to the front)
        while not self.robot_object.command_packets.empty():
            command = self.robot_object.command_packets.get()

            # TODO: add command logging
            # if self.log_data and self.logger.time0 != 0:
            #     self.logger.record(self.whoiam, command)

            if not self.write_packet(command):
                self.stop()
                raise RobotSerialPortWritePacketError("Failed to send command:", command)

    def parse_packets(self, packets):
        # parse each packet according to its receive method
        for packet in packets:
            try:
                # Parse the packet using robot_object's receive method
                self.robot_object.receive(packet)
            except:
                self.stop()
                raise ReceivePacketError(
                    "Robot object's receive method threw an exception\n"
                    "Received packets:\n%s\n"
                    "Error packet: %s" % (
                        pprint.pformat(packets), repr(packet)
                    ), self
                )

            if self.log_data and self.logger.time0 != 0:
                self.logger.record(self.whoiam, packet)

            # if any packets were received, signal the object updated
            self.robot_object._updated = True

    def read_packets(self):
        """
        Read all available data on serial and split them into packets as
        indicated by packet_end.

        :return: False indicates the serial read failed and that the communicator thread should be stopped.
        """
        try:
            # read every available character
            incoming = self.serial_ref.read(self.serial_ref.in_waiting)
        except SerialException as error:
            self.handle_error(error)
            return [], False
        except portNotOpenError as error:
            self.handle_error(error)
            return [], False

        if len(incoming) > 0:
            # append to the buffer
            try:
                self.buffer += incoming.decode('ascii')
            except UnicodeDecodeError as error:
                self.handle_error(error)
                return [], False

            if len(self.buffer) > len(self.packet_end):
                # split based on user defined packet end
                packets = self.buffer.split(self.packet_end)

                # reset the buffer
                self.buffer = packets.pop(-1)

                return packets, True
        return [], True

    def write_packet(self, packet):
        """
        Safely write a byte over serial. Automatically appends packet_end to the input
        This method is called by the thread if the robot_object indicates a new packet is available
        should_stop is set to False if the write fails

        :param packet: an arbitrary string without packet_end in it
        :return: None
        """
        try:
            data = bytearray(packet + self.packet_end, 'ascii')
        except TypeError as error:
            self.handle_error(error)
            return False

        try:
            self.serial_ref.write(data)
        except SerialException as error:
            self.handle_error(error)
            return False
        except portNotOpenError as error:
            self.handle_error(error)
            return False

        return True

    def print_packets(self, packets):
        """
        If debug_prints is True, print repr of all incoming packets
        :param packets: a list of received packets
        :return: None
        """
        if self.debug_prints:
            for packet in packets:
                print("> %s" % repr(packet))

    # ----- external and status methods -----

    def assign_robot_object(self, robot_object):
        """
        After the whoiam ID is discovered, assign the robot_object with the corresponding whoiam ID
        :param robot_object: An instance of the RobotObject class
        :return: None
        """
        self.robot_object = robot_object

        # ensures properties being set by robot objects aren't being
        # accessed by the port at the same time

        self.port_assigned = True

    def parse_first_packet(self):
        # send the first packet received to the object
        try:
            if len(self.first_packet) > 0:
                self.robot_object.receive_first(self.first_packet)
            elif self.debug_prints:
                print("%s, '%s': no data found accompanying whoiam packet" % (self.address, self.whoiam))
            return True
        except Exception as error:
            self.handle_error(error)
            return False

    def is_running(self):
        """
        Check if the port's thread is running correctly. This will be False if:
         should_stop is False
         The time recorded by the thread and the main thread don't sync up
            (this method is meant to be called from the main thread)
        :return: bool
        """
        if not self.configured:
            return 0

        current_time = int(time.time() - self.start_time)
        status = abs(current_time - self.thread_time) < 2
        if not status:
            return -1
        return 1

    def stop(self):
        """
        Send stop packet, set should_stop to True, close the serial port.
        Don't wait for feedback from the microcontroller. There's some weird race condition going on
        :return: None
        """
        if not self.stop_sent:
            self.stop_sent = True

            self.configured = False

            time.sleep(0.05)
            self.write_packet("stop\n")

            if self.debug_prints:
                print("Sent stop flag")
            time.sleep(0.05)

            self.serial_ref.close()

            if self.debug_prints:
                print("Closing serial")


class RobotObject:
    def __init__(self, whoiam):
        self.whoiam = whoiam
        self._updated = False

        self.command_packets = Queue(maxsize=255)

    def receive_first(self, packet):
        """
        Overwrite this method when subclassing RobotObject if you're expecting initial data

        Initialize any data here. If the who_am_i packet contains data, it's passed here
        :param packet: The first packet received by the robot object's port
        :return: None
        """
        pass

    def receive(self, packet):
        """
        Overwrite this method when subclassing RobotObject

        Parse incoming packets received by the corresponding port.
        This method is called on the RobotSerialPort's thread.
        I would recommend ONLY parsing packets here and not doing anything else.
        Use did_update for any event based function calls in the main thread.

        :param packet: A packet (string) received from the robot object's port
        :return: None
        """
        pass

    def send(self, packet):
        """
        Do NOT overwrite this method when subclassing RobotObject

        Queue a new packet for sending
        :param packet: A packet (string) to send to the microcontroller
        :return: None
        """
        self.command_packets.put(packet)

    def did_update(self):
        """
        Returns True if the corresponding RobotSerialPort object received new packets
        :return: bool
        """
        if self._updated:
            self._updated = False
            return True
        else:
            return False


class RobotInterface:
    def __init__(self, *robot_objects, joystick=None,
                 log_data=True, log_name=None, log_dir=None, debug_prints=False):
        """
        :param robot_objects: subclasses of RobotObject
        :param joystick: A subclass instance of BuggyJoystick
        :param log_data: A boolean indicating whether or not the received data should be written to a log file
        :param log_name: The name of the log file. If None, it will be today's date and time
        :param log_dir: The directory of the log file. If None, it will be today's date.
            See project.py for details
        :param debug_prints: Enable verbose prints
        """
        self.debug_prints = debug_prints

        self.log_data = log_data
        if self.log_data:
            # if no directory provided, use the ":today" flag
            # (follows the project.py directory reference convention)
            if log_dir is None:
                log_dir = ":today"
            self.logger = Logger(log_name, log_dir)
        else:
            self.logger = None

        self.joystick = joystick

        self.ports = {}
        self.objects = {}
        for robot_object in robot_objects:
            self.objects[robot_object.whoiam] = robot_object

        # open all available ports
        for port_info in serial.tools.list_ports.comports():
            if port_info.serial_number is not None:
                port = RobotSerialPort(port_info, self.logger, log_data, self.debug_prints)
                self.ports[port.whoiam] = port

                if not port.configured:
                    self._close_ports()
                    self.print_port_info(port)

                    raise RobotSerialPortNotConfiguredError("Port not configured!", port)

        # assign ports based on identity
        for whoiam in self.objects.keys():
            if whoiam in self.ports.keys():
                # Assign the robot_object to the corresponding port
                self.ports[whoiam].assign_robot_object(self.objects[whoiam])

                # signal first packet event
                success = self.ports[whoiam].parse_first_packet()
                if not success:
                    self._close_ports()
                    self.print_port_info(self.ports[whoiam])

                    raise ReceivePacketError("Failed to parse the first packet.", self.ports[whoiam])
            else:
                self._close_ports()
                self.print_port_info(self.ports[whoiam])

                raise RobotObjectNotFoundError("Failed to assign robot object with ID '%s'" % whoiam)

        for whoiam in self.ports.keys():
            if not self.ports[whoiam].port_assigned:
                self._close_ports()
                self.print_port_info(self.ports[whoiam])

                RobotSerialPortUnassignedError("Port not assigned to an object.", self.ports[whoiam])

    def print_port_info(self, port):
        if self.debug_prints:
            pprint.pprint(port.port_info.__dict__)
            print()
            time.sleep(0.01)

    @property
    def dt(self):
        if self.logger.log_started:
            return time.time() - self.logger.time0
        else:
            return 0.0

    def _start_ports(self):
        for robot_port in self.ports.values():
            robot_port.start()

        if self.log_data:
            self.logger.start_time()

    def _close_ports(self):
        """
        Kill all RobotSerialPort threads and close their serial ports
        :return: None
        """
        if self.debug_prints:
            print("Closing all ports")
        for robot_port in self.ports.values():
            robot_port.stop()

    def loop(self):
        """
        Similar to Arduino's loop function. This will be run in a while True loop
        :return:
        """
        pass

    def close(self):
        pass

    def _are_ports_active(self):
        for robot_port in self.ports.values():
            status = robot_port.is_running()
            if status < 1:
                self._close_ports()
                if status == 0:
                    raise RobotSerialPortSignalledExitError("Port with ID '%s' signalled to exit" % robot_port.whoiam)
                elif status == -1:
                    raise RobotSerialPortTimeoutError("Port with ID '%s' timed out" % robot_port.whoiam)
        return True

    def run(self):
        """
        Call this method to start the robot and to start receiving data

        This is where all threads start and stop. Stop events can be thrown by:
            loop returning False (live plot in the loop returned False for example)
            joystick returned False, the pygame window signaled to quit
            are_threads_running() returned False

        are_threads_running can be False for the following reasons:
            robot_port.is_running found that the serial thread has been hanging
                for more than 2 seconds
            robot_port.is_running found that should_stop is False

        should_stop can be False because:
            A packet wasn't parsed correctly
            The port never signaled ready
            There weren't enough ports to match the number of objects
            No ports with the requested whoiam ID were found
            The port wasn't matched to a RobotObject (it was unused)
        :return:
        """
        self._start_ports()

        try:
            while self._are_ports_active():
                if self.loop() is False:
                    if self.debug_prints:
                        print("loop signaled to exit")
                    break

                if self.joystick is not None:
                    if self.joystick.update() is False:
                        break
        except KeyboardInterrupt:
            pass

        self.close()
        self._close_ports()
