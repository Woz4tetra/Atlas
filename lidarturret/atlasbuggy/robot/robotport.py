"""
RobotSerialPorts are the direct interface between serial port objects and robot objects.
Each port runs on its own process. The whoiam ID of the port is found at runtime. The
corresponding robot object is paired in RobotInterface.
"""

import time
import traceback
from multiprocessing import Event, Process

import serial
import serial.tools.list_ports
from serial.serialutil import SerialException

from atlasbuggy.robot.clock import Clock
from atlasbuggy.robot.errors import *


class RobotSerialPort(Process):
    """
    A multiprocessing based wrapper for an instance of pyserial Serial.
    A RobotInterface class manages all instances of this class.
    This class is for internal use only
    """

    def __init__(self, port_info, debug_prints, queue, lock, counter, updates_per_second):
        """

        :param port_info: A ListPortInfo object returned by serial.tools.list_ports.comports()
        :param debug_prints: Enable verbose print statements
        :param queue: a multiprocessing queue to which packets are passed
        :param lock: a shared lock to prevent multiple sources accessing the queue
        :param counter: a queue size counter. Keeps track of the number of packets in the queue
        :param updates_per_second: How often the port should update. This is passed to a Clock instance
        """

        # port info variables
        self.address = port_info.device  # directory to open (in /dev)
        self.port_info = port_info

        # status variables
        self.debug_prints = debug_prints
        self.configured = True
        self.error_message = None
        self.port_assigned = False

        # time variables
        self.start_time = 0.0
        self.thread_time = 0
        self.loop_time = 0.0
        self.updates_per_second = updates_per_second

        # whoiam ID info
        self.whoiam = None  # ID tag of the microcontroller
        self.whoiam_header = "iam"  # whoiam packets start with "iam"
        self.whoiam_ask = "whoareyou"

        # first packet info
        self.first_packet = None
        self.first_packet_ask = "init?"
        self.first_packet_header = "init:"

        # misc. serial protocol
        self.packet_end = "\n"  # what this microcontroller's packets end with
        self.baud_rate = 115200

        # buffer for putting packets into
        self.buffer = ""

        # variable to signal exit
        self.exit_event = Event()

        # attempt to open the serial port
        try:
            self.serial_ref = serial.Serial(port=self.address, baudrate=self.baud_rate)
        except SerialException as error:
            self.handle_error(error)

        time.sleep(2)  # wait for microcontroller to wake up

        if self.configured:
            # Find the ID of this port. The ports will be matched up to the correct RobotObject later
            self.find_whoiam()
            if self.whoiam is not None:
                self.find_first_packet()
            else:
                self.debug_print("whoiam ID was None, skipping find_first_packet")

        else:
            self.debug_print("Port not configured. Skipping find_whoiam")

        super(RobotSerialPort, self).__init__(target=self.update, args=(queue, lock, counter))

    # ----- initialization methods -----

    def send_start(self):
        """
        Send the start flag
        :return: None
        """
        self.write_packet("start")

    def find_whoiam(self):
        """
        Get the whoiam packet from the microcontroller. This method will wait 1 second
        until the packet is received

        example:
            sent: "whoareyou\n"
            received: "iamlidar\n"

        When the packet is found, parse_whoiam_packet is called and whoiam is assigned
        :return: whoiam packet and first_packet
        """

        self.whoiam = self.check_protocol(self.whoiam_ask, self.whoiam_header)

        if self.whoiam is not None:
            self.debug_print("%s has ID '%s'" % (self.address, self.whoiam))
        else:
            self.debug_print("Failed to obtain whoiam ID!", ignore_flag=True)

    def find_first_packet(self):
        """
        Get the first packet from the microcontroller. This method will wait 1 second
        until the packet is received

        example:
            sent: "init?\n"
            received: "init:\n" (if nothing to init, initialization methods not called)
            received: "init:something interesting\t01\t23\n"
                'something interesting\t01\t23' would be the first packet

        When the packet is found, parse_whoiam_packet is called and whoiam is assigned
        :return: whoiam packet and first_packet
        """
        self.first_packet = self.check_protocol(self.first_packet_ask, self.first_packet_header)

        if self.first_packet is not None:
            self.debug_print("sent initialization data: %s" % repr(self.first_packet))
        else:
            self.debug_print("Failed to obtain first packet!", ignore_flag=True)

    def check_protocol(self, ask_packet, recv_packet_header):
        """
        A call and response method. After an "ask packet" is sent, the process waits for
        a packet with the expected header for 2 seconds

        :param ask_packet: packet to send
        :param recv_packet_header: what the received packet should start with
        :return: the packet received without the header and packet end
        """
        self.debug_print("Checking '%s' protocol" % ask_packet)

        if not self.write_packet(ask_packet):
            return None  # return None if write failed

        start_time = time.time()
        abides_protocol = False
        answer_packet = ""

        # wait for the correct response
        while not abides_protocol:
            packets = self.read_packets()
            self.print_packets(packets)

            # return None if read failed
            if packets is None:
                self.handle_error("Serial read failed... Board never signalled ready")
                return None

            # return None if operation timed out
            if (time.time() - start_time) > 2:
                self.handle_error("Didn't receive response for packet '%s'. Operation timed out." % ask_packet)
                return None

            # parse received packets
            for packet in packets:
                if packet[0:len(recv_packet_header)] == recv_packet_header:  # if the packet starts with the header,
                    self.debug_print("received packet: " + repr(packet))

                    answer_packet = packet[len(recv_packet_header):]  # record it and return it

                    self.debug_print("answer packet: " + repr(answer_packet))
                    abides_protocol = True

        self.debug_print("returning answer packet:" + repr(answer_packet))
        return answer_packet  # when the while loop exits, abides_protocol must be True

    def handle_error(self, error):
        """
        When errors occur in a RobotSerialPort, the process doesn't crash. The error is recorded,
        self.update is stopped, and the main process is notified so all other ports can close safely
        :param error: The error message to record
        :return: None
        """
        self.configured = False
        if self.error_message is None:
            self.error_message = traceback.format_stack()
            if type(error) == str:
                full_message = error
            else:
                full_message = "%s: %s" % (error.__class__.__name__, str(error))
            self.error_message.append(full_message)

    # ----- run methods -----

    def update(self, queue, lock, counter):
        """
        Called when RobotSerialPort.start is called

        :param queue: A reference to the queue to pass data to
        :param lock: The packet queue lock
        :param counter: Number of packets
        :return: None
        """

        self.start_time = time.time()
        clock = Clock(self.updates_per_second)
        clock.start(self.start_time)

        try:
            while not self.exit_event.is_set():
                # update the internal timer. Acts as a check to see if the thread is running properly
                self.thread_time = int(time.time() - self.start_time)

                # close the process if the serial port isn't open
                if not self.serial_ref.is_open:
                    self.stop()
                    self.close_port()
                    raise RobotSerialPortClosedPrematurelyError("Serial port isn't open for some reason...")

                if self.serial_ref.in_waiting > 0:
                    # read every possible character available and split them into packets
                    packets = self.read_packets()
                    if packets is None:  # if the read failed
                        self.stop()
                        self.close_port()
                        raise RobotSerialPortReadPacketError("Failed to read packets")

                    # put data found into the queue
                    with lock:
                        for packet in packets:
                            queue.put((self.whoiam, time.time(), packet))
                            # start_time isn't used. The main process has its own initial time reference

                        counter.value += len(packets)

                clock.update()  # maintain a constant loop speed

        except KeyboardInterrupt:
            self.debug_print("KeyboardInterrupt in port loop")

        self.debug_print("While loop exited. Exit event triggered. Closing port")
        self.close_port()

    def read_packets(self):
        """
        Read all available data on serial and split them into packets as
        indicated by packet_end.

        :return: None indicates the serial read failed and that the communicator thread should be stopped.
            returns the received packets otherwise
        """
        try:
            # read every available character
            if self.serial_ref.is_open:
                incoming = self.serial_ref.read(self.serial_ref.in_waiting)
            else:
                self.handle_error("Serial port wasn't open for reading...")
                return None
        except SerialException as error:
            self.handle_error(error)
            return None

        if len(incoming) > 0:
            # append to the buffer
            try:
                self.buffer += incoming.decode('ascii')
            except UnicodeDecodeError as error:
                self.handle_error(error)
                return None

            if len(self.buffer) > len(self.packet_end):
                # split based on user defined packet end
                packets = self.buffer.split(self.packet_end)

                # reset the buffer
                self.buffer = packets.pop(-1)

                return packets
        return []

    def write_packet(self, packet):
        """
        Safely write a packet over serial. Automatically appends packet_end to the input.

        :param packet: an arbitrary string without packet_end in it
        :return: True or False if the write was successful
        """
        try:
            data = bytearray(packet + self.packet_end, 'ascii')
        except TypeError as error:
            self.handle_error(error)
            return False

        try:
            if self.serial_ref.is_open:
                self.serial_ref.write(data)
            else:
                self.handle_error("Serial port wasn't open for writing...")
                return False
        except SerialException as error:
            self.handle_error(error)
            return False

        return True

    def print_packets(self, packets):
        """
        If debug_prints is True, print repr of all incoming packets
        :param packets: a list of received packets
        :return: None
        """
        for packet in packets:
            self.debug_print("> %s" % repr(packet))

    # ----- external and status methods -----

    def debug_print(self, string, ignore_flag=False):
        if self.debug_prints or ignore_flag:
            print("[%s] %s" % (self.address, string))

    def is_running(self):
        """
        Check if the port's thread is running correctly

        :return:
            -1: timeout error
            0: self.configured is False
            1: process hasn't started or everything is fine
        """
        if not self.configured:
            return 0

        if self.start_time == 0.0:  # process hasn't started
            return 1

        current_time = int(time.time() - self.start_time)
        status = abs(current_time - self.thread_time) < 2
        if not status:
            return -1
        else:
            return 1

    def stop(self):
        """
        Send stop packet, close the serial port.
        Don't wait for feedback from the microcontroller
        :return: None
        """

        if self.configured:
            self.write_packet("stop\n")
            self.debug_print("Sent stop flag")
            self.configured = False

        self.exit_event.set()

    def close_port(self):
        """
        Close the serial port if it's open
        :return: None
        """
        if self.serial_ref.is_open:
            self.serial_ref.close()
            self.debug_print("Closing serial")
        else:
            print("Serial port was already closed!")
