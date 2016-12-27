import glob
import time
import traceback
from threading import Thread, Lock
from queue import Queue

import serial

from robot.analysis import Logger
import project


class RobotSerialPort(Thread):
    opened_addresses = []

    def __init__(self, baud, address_format, packet_end, logger, log_data):
        self.address = None  # final selected address
        self.address_format = None  # pattern of address to look for. Assigned later
        self.serial_ref = None  # pyserial Serial class reference, assigned when connection is made

        self.who_i_am = None  # ID tag of the microcontroller
        self.who_i_am_header = "iam"  # who_i_am packets start with "iam"
        self.packet_end = packet_end  # what this microcontroller's packets end with
        assert len(self.packet_end) > 0

        # object to pass packets to for parsing, set externally after all
        # discovered ports have been opened
        self.robot_object = None

        # buffer for putting packets into
        self.buffer = ""
        self.first_packet = ""

        self.should_stop = False

        self.log_data = log_data
        self.logger = logger  # reference to instance of the Logger class

        # prevents data from being accessed from main and serial thread at the same time
        self.property_lock = Lock()

        self.platform = project.get_platform()

        # if address_format provided as a dictionary, assume the keys are
        # platform types (platform specific address formats)
        if type(address_format) == dict:
            if self.platform in address_format.keys():
                address_format = address_format[self.platform]
            else:
                raise ValueError("Platform specific address not specified for",
                                 self.platform)

        # if address_format is given as a single string, wrap it in a list for
        # convenience of the for loop
        if type(address_format) == str:
            self.address_format = [address_format]

        # if invalid data type for address_format is provided, throw error
        if self.address_format is None:
            raise ValueError("Invalid address format:", address_format)

        # from address_format, generate possible addresses
        for address in self.parse_addresses(self.address_format):
            if address not in RobotSerialPort.opened_addresses:
                # attempt to open the serial port
                try:
                    self.serial_ref = \
                        serial.Serial(port=address, baudrate=baud)
                except serial.SerialException:
                    pass

                # for the first port that succeeds, break
                self.address = address
                RobotSerialPort.opened_addresses.append(self.address)
                break

        # if no addresses could connect, throw error
        if self.address is None:
            raise ConnectionError(
                "Could not find serial port... %s" % address_format)

        print("Using", self.address)

        # Find the ID of this port. We didn't check this while opening the port
        # because the correct ports will be matched up to the correct
        # RobotObject later
        self.find_who_i_am()

        super(RobotSerialPort, self).__init__()

    def parse_addresses(self, address_format):
        """
        Using the glob model, parse the address format into a list of
        real directories
        """
        addresses = []
        for address in address_format:
            addresses.extend(glob.glob(address))
        return addresses

    def run(self):
        try:
            while not self.should_stop:
                # if the serial port is still active
                if self.serial_ref.is_open and self.serial_ref.in_waiting > 0:
                    # read every possible character available and split them
                    # into packets
                    packets, status = self.read_packets()
                    if not status:
                        raise Exception("Packet read failed...")

                    # unlock shared properties for this thread (pauses access
                    # activity in other threads)
                    self.property_lock.acquire()

                    # parse each packet according to its receive method
                    for packet in packets:
                        self.robot_object.receive(packet)
                        if self.log_data:
                            self.logger.record(self.who_i_am, packet)

                    # if any packets were received, signal the object updated
                    if len(packets) > 0:
                        self.robot_object.updated = True

                    # to avoid an endless loop of objects having references to
                    # ports and ports to objects, the port just checks if the
                    # object updated its command packet and sends it

                    # dequeue all command packets and write them (send from the
                    # end to the front)
                    while not self.robot_object.command_packets.empty():
                        self.write_packet(
                            self.robot_object.command_packets.get())

                    # release shared property lock
                    self.property_lock.release()
        except:
            traceback.print_exc()
            self.stop()

    def read_packets(self):
        """
        Read all available data on serial and split them into packets as
        indicated by packet_end. If serial crashes, the method
        returns False indicating that the communicator thread should be stopped.
        """
        try:
            # read every available character
            incoming = self.serial_ref.read(self.serial_ref.in_waiting)
            if len(incoming) > 0:
                # append to the buffer
                self.buffer += incoming.decode('ascii')
                # split based on user defined packet end
                packets = self.buffer.split(self.packet_end)

                if len(self.buffer) > len(self.packet_end):
                    # reset the buffer
                    self.buffer = packets.pop(-1)
                return packets, True
            else:
                return [], True
        except:
            print("Exception occurred during serial read")
            traceback.print_exc()
            self.stop()
            return [], False

    def stop(self):
        # send stop packet. Don't wait for feedback. There's some weird race
        # condition going on
        time.sleep(0.05)
        self.write_packet("stop\n")
        time.sleep(0.05)
        self.should_stop = True
        self.serial_ref.close()

    def write_packet(self, packet):
        """
        Safely write a byte over serial. Automatically appends packet_end to
        the input
        """
        data = bytearray(packet + self.packet_end, 'ascii')
        if not self.should_stop:
            try:
                self.serial_ref.write(data)
            except:
                print("Serial write failed...")
                self.should_stop = True

    def parse_who_i_am_packet(self, packet):
        # if packet contains who_i_am_header
        if packet[0:len(self.who_i_am_header)] == self.who_i_am_header:
            # remove the "iam" header
            packet = packet[len(self.who_i_am_header):]

            # who_i_am packets can contain initialization data, this is
            # indicated by a tab character. If it exists, assign the rest of
            # the packet to first_packet
            data_index = packet.find("\t")
            if data_index > 0:
                who_i_am = packet[:data_index]
                first_packet = packet[data_index + 1:]
            else:
                who_i_am = packet
                first_packet = ""

            return who_i_am, first_packet
        else:
            return None, ""

    def print_packets(self, packets):
        for packet in packets:
            print("> '%s'" % packet)

    def find_who_i_am(self):
        time.sleep(0.005)
        # reset or wake the microcontroller
        self.write_packet("ready?")
        packets, status = self.read_packets()
        self.print_packets(packets)

        # wait for the correct response
        while "ready!" not in packets:
            if not status:
                raise Exception(
                    "Serial read failed... Board never signaled ready")
            packets, status = self.read_packets()
            self.print_packets(packets)

        print(self.address, "is ready!")

        # ask for who_i_am packet
        self.write_packet("whoareyou")

        # wait for correct response
        while self.who_i_am is None:
            packets, status = self.read_packets()
            self.print_packets(packets)
            if not status:
                raise Exception("Serial read failed... "
                                "Couldn't get a who_am_i packet.")
            # parse received packets. parse_who_i_am_packet determines if any
            # of the packets are valid
            for packet in packets:
                self.who_i_am, self.first_packet = \
                    self.parse_who_i_am_packet(packet)

                if self.who_i_am is not None:
                    break
            time.sleep(0.1)

        # record the first packet if it exists
        if self.log_data and len(self.first_packet) > 0:
            self.logger.record(self.who_i_am, self.first_packet)

        print(self.address, "is", self.who_i_am)


class RobotObject:
    def __init__(self, who_i_am, address_format, baud=115200):
        self.who_i_am = who_i_am
        self.address_format = address_format
        self.baud = baud
        self.property_lock = None
        self.updated = False

        self.command_packets = Queue()

        # to prevent two threads accessing data at the same time, overwrite
        # set to include property_lock. This is a hacky way to avoid having to
        # set all properties through an arbitrary set function
        self.__setattr__ = self.setattr

    def setattr(self, key, value):
        self.property_lock.acquire()
        object.__setattr__(self, key, value)
        self.property_lock.release()

    def receive_first(self, packet):
        # initialize any data here. If the who_am_i packet contains data,
        # it's passed here
        pass

    def receive(self, packet):
        # parse incoming packets received by the corresponding port
        pass

    def send(self, packet):
        # queue the new packet for sending
        self.command_packets.put(packet)

    def did_update(self):
        if self.updated:
            self.updated = False
            return True
        else:
            return False


class RobotInterface:
    def __init__(self, *robot_objects, baud_rate=115200, packet_end='\n',
                 log_data=True, log_name=None, log_dir=None):
        self.log_data = log_data
        if self.log_data:
            # if no directory provided, use the ":today" flag (follows the
            # project.py directory reference convention)
            if log_dir is None:
                log_dir = ":today"
            self.logger = Logger(log_name, log_dir)
        else:
            self.logger = None

        # open all available ports
        self.ports = {}
        self.objects = {}
        for robot_object in robot_objects:
            port = RobotSerialPort(baud_rate, robot_object.address_format,
                                   packet_end, self.logger, log_data)
            self.ports[port.who_i_am] = port
            self.objects[robot_object.who_i_am] = robot_object

        # assign ports based on identity
        for who_i_am in self.ports.keys():
            self.ports[who_i_am].robot_object = self.objects[who_i_am]

            # ensures properties being set by robot objects aren't being
            # accessed by the port at the same time
            self.objects[who_i_am].property_lock = \
                self.ports[who_i_am].property_lock

            # send the first packet received to the object
            self.objects[who_i_am].receive_first(
                self.ports[who_i_am].first_packet
            )

    def _start(self):
        for robot_port in self.ports.values():
            robot_port.start()

        if self.log_data:
            self.logger.start_time()

    def _stop(self):
        for robot_port in self.ports.values():
            robot_port.stop()

    def loop(self):
        pass

    def close(self):
        pass

    def run(self):
        self._start()
        try:
            while True:
                self.loop()
        except:
            traceback.print_exc()
        finally:
            self.close()
            self._stop()
