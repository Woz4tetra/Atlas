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
    A multi-threaded wrapper for an instance of pyserial Serial.
    A RobotInterface class manages all instances of this class.
    This class is for internal use only
    """

    def __init__(self, port_info, debug_prints, queue, lock, counter, updates_per_second):
        """
        :param port_info: A ListPortInfo object returned by serial.tools.list_ports.comports()
        :param debug_prints: Enable verbose print statements
        """
        self.address = port_info.device  # directory to open (in /dev)
        self.port_info = port_info

        # status variables
        self.debug_prints = debug_prints
        self.configured = True
        self.error_message = None
        self.port_assigned = False

        self.start_time = 0.0
        self.thread_time = 0
        self.loop_time = 0.0
        self.updates_per_second = updates_per_second

        self.whoiam = None  # ID tag of the microcontroller
        self.whoiam_header = "iam"  # whoiam packets start with "iam"
        self.whoiam_ask = "whoareyou"

        self.first_packet = None
        self.first_packet_ask = "init?"
        self.first_packet_header = "init:"

        self.packet_end = "\n"  # what this microcontroller's packets end with
        self.baud_rate = 115200

        # buffer for putting packets into
        self.buffer = ""

        # attempt to open the serial port
        try:
            self.serial_ref = serial.Serial(port=self.address, baudrate=self.baud_rate)
        except SerialException as error:
            self.handle_error(error)

        if self.configured:
            # Find the ID of this port. The ports will be matched up to the correct RobotObject later
            self.find_whoiam()
            if self.whoiam is not None:
                self.find_first_packet()
            elif self.debug_prints:
                print("whoiam ID was None, skipping find_first_packet")
        elif self.debug_prints:
            print("Port not configured. Skipping find_whoiam")

        self.exit_event = Event()
        super(RobotSerialPort, self).__init__(target=self.update, args=(queue, lock, counter))

    # ----- initialization methods -----

    def find_whoiam(self):
        """
        Get the whoiam packet from the microcontroller. This method will wait 1 second
        until the packet is received

        example: "iamlidar"

        When the packet is found, parse_whoiam_packet is called and whoiam is assigned
        :return: whoiam packet and first_packet
        """

        self.whoiam = self.check_protocol(self.whoiam_ask, self.whoiam_header)

        if self.debug_prints:
            if self.whoiam is not None:
                print("%s has ID '%s'" % (self.address, self.whoiam))
            else:
                print("Failed to obtain whoiam ID!")

    def find_first_packet(self):
        self.first_packet = self.check_protocol(self.first_packet_ask, self.first_packet_header)

        if self.debug_prints:
            if self.first_packet is not None:
                print("%s sent initialization data: %s" % (self.address, repr(self.first_packet)))
            else:
                print("Failed to obtain first packet!")

    def check_protocol(self, ask_packet, recv_packet_header):
        if self.debug_prints:
            print("Checking '%s' protocol" % ask_packet)
        if not self.write_packet(ask_packet):
            return None

        start_time = time.time()
        abides_protocol = False
        answer_packet = ""

        # wait for the correct response
        while not abides_protocol:
            packets, status = self.read_packets()
            self.print_packets(packets)
            if not status:
                self.handle_error("Serial read failed... Board never signalled ready")
                return None
            if (time.time() - start_time) > 2:
                self.handle_error("Didn't receive response for packet '%s'. Operation timed out." % ask_packet)
                return None

            for packet in packets:
                if packet[0:len(recv_packet_header)] == recv_packet_header:
                    if self.debug_prints:
                        print("received packet:", repr(packet))
                    answer_packet = packet[len(recv_packet_header):]
                    if self.debug_prints:
                        print("answer packet:", repr(answer_packet))
                    abides_protocol = True

        if self.debug_prints:
            print("returning answer packet:", repr(answer_packet))
        return answer_packet  # when the while loop exits, abides_protocol must be True

    def handle_error(self, error):
        self.configured = False
        if self.error_message is None:
            self.error_message = traceback.format_stack()
            self.error_message.append("%s: %s" % (error.__class__.__name__, str(error)))

    # ----- run methods -----

    def update(self, queue, lock, counter):
        """Called when RobotSerialPort.start is called (inherited from threading.Thread)"""

        self.start_time = time.time()
        clock = Clock(self.updates_per_second)
        clock.start()

        try:
            while not self.exit_event.is_set():
                # update the internal timer. Acts as a check to see if the thread is running properly
                self.thread_time = int(time.time() - self.start_time)
                if not self.serial_ref.is_open:
                    self.stop()
                    self.close_port()
                    raise RobotSerialPortClosedPrematurelyError("Serial port isn't open for some reason...")

                if self.serial_ref.in_waiting > 0:
                    # read every possible character available and split them into packets
                    packets, status = self.read_packets()
                    if not status:
                        self.stop()
                        self.close_port()
                        raise RobotSerialPortReadPacketError("Failed to read packets")

                    with lock:
                        for packet in packets:
                            queue.put((self.whoiam, time.time(), packet))
                        counter.value += len(packets)

                clock.update()

        except KeyboardInterrupt:
            if self.debug_prints:
                print("KeyboardInterrupt in port loop")

        if self.debug_prints:
            print("While loop exited. Exit event triggered. Closing port")
        self.close_port()

    def read_packets(self):
        """
        Read all available data on serial and split them into packets as
        indicated by packet_end.

        :return: False indicates the serial read failed and that the communicator thread should be stopped.
        """
        try:
            # read every available character
            if self.serial_ref.is_open:
                incoming = self.serial_ref.read(self.serial_ref.in_waiting)
            else:
                self.handle_error("Serial port wasn't open for reading...")
                return [], False
        except SerialException as error:
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
        if self.debug_prints:
            for packet in packets:
                print("> %s" % repr(packet))

    # ----- external and status methods -----

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
        Send stop packet, set should_stop to True, close the serial port.
        Don't wait for feedback from the microcontroller. There's some weird race condition going on
        :return: None
        """

        if self.configured:
            self.write_packet("stop\n")

            if self.debug_prints:
                print("Sent stop flag")

            self.configured = False

        self.exit_event.set()

    def close_port(self):
        if self.serial_ref.is_open:
            self.serial_ref.close()

            if self.debug_prints:
                print("Closing serial")
