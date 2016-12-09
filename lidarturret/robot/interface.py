import glob
import time
import traceback
from threading import Thread, Lock

import serial

from robot.analysis import Logger
import project


class RobotSerialPort(Thread):
    opened_addresses = []

    def __init__(self, baud, address_format, packet_end, logger, log_data):
        self.address = None
        self.address_format = None
        self.serial_ref = None

        self.who_i_am = None
        self.who_i_am_header = "iam"
        self.packet_end = packet_end

        self.robot_object = None

        self.property_lock = Lock()

        self.platform = project.get_platform()

        if type(address_format) == dict:
            if self.platform in address_format.keys():
                address_format = address_format[self.platform]
            else:
                raise ValueError("Platform specific address not specified for",
                                 self.platform)

        if type(address_format) == str:
            self.address_format = [address_format]

        if self.address_format is None:
            raise ValueError("Invalid address format:", address_format)

        for address in self.parse_addresses(self.address_format):
            if address not in RobotSerialPort.opened_addresses:
                try:
                    self.serial_ref = \
                        serial.Serial(port=address,
                                      baudrate=baud,
                                      timeout=0.005)
                except serial.SerialException:
                    pass
                self.address = address
                RobotSerialPort.opened_addresses.append(self.address)
                break

        if self.address is None:
            raise ConnectionError(
                "Could not find serial port... %s" % address_format)
        print("Using", self.address)

        self.buffer = ""
        self.should_stop = False

        self.find_who_i_am()

        self.log_data = log_data
        self.logger = logger

        super(RobotSerialPort, self).__init__()

    def parse_addresses(self, address_format):
        addresses = []
        for address in address_format:
            addresses.extend(glob.glob(address))
        return addresses


    def run(self):
        try:
            while not self.should_stop:
                self.property_lock.acquire()

                if self.serial_ref.is_open and self.serial_ref.in_waiting > 0:
                    packets, status = self.read_packets()
                    if not status:
                        raise Exception("Packet read failed...")

                    if len(packets) > 0:
                        self.robot_object.updated = True
                    for packet in packets:
                        self.robot_object._receive(packet)
                        if self.log_data:
                            self.logger.record(self.who_i_am, packet)

                if self.robot_object.has_new_command:
                    self.write_packet(self.robot_object.command_packet +
                                      self.packet_end)
                    self.robot_object.has_new_command = False
                self.property_lock.release()
        except:
            traceback.print_exc()
            self.stop()

    def read_packets(self):
        """
        Read all available data on serial and split them into packets as
        indicated by the characters \r\n. If serial crashes, the method
        returns False indicating that the communicator thread should be stopped.
        """
        try:
            incoming = self.serial_ref.read(self.serial_ref.in_waiting)
            if len(incoming) > 0:
                self.buffer += incoming.decode('ascii')
                packets = self.buffer.split(self.packet_end)

                if self.buffer[-2:] != '\n':
                    self.buffer = packets.pop(-1)
                else:
                    self.buffer = ""
                return packets, True
            else:
                return [], True
        except:
            print("Exception occurred during serial read")
            traceback.print_exc()
            self.stop()
            return [], False

    def stop(self):
        time.sleep(0.05)
        self.write_packet("stop\n")
        time.sleep(0.05)
        self.should_stop = True
        self.serial_ref.close()

    def write_packet(self, packet):
        """Safely write a byte over serial"""
        data = bytearray(packet, 'ascii')
        if not self.should_stop:
            try:
                self.serial_ref.write(data)
            except:
                print("Serial write failed...")
                self.should_stop = True

    def parse_who_i_am_packet(self, packet):

        if packet[0:len(self.who_i_am_header)] == self.who_i_am_header:
            # the rest of the packet is the ID number
            return packet[len(self.who_i_am_header):]
        else:
            return None

    def find_who_i_am(self):
        time.sleep(0.005)

        self.write_packet("ready?\n")
        packets, status = self.read_packets()
        while "ready!" not in packets:
            if not status:
                raise Exception(
                    "Serial read failed... Board never signaled ready")
            packets, status = self.read_packets()

        print(self.address, "is ready!")

        self.write_packet("whoareyou\n")

        while self.who_i_am is None:
            packets, status = self.read_packets()
            if not status:
                raise Exception("Serial read failed... Couldn't get board ID")
            for packet in packets:
                self.who_i_am = self.parse_who_i_am_packet(packet)
                if self.who_i_am is not None:
                    break
            time.sleep(0.1)

        print(self.address, "is", self.who_i_am)


class RobotObject:
    def __init__(self, who_i_am, address_format, baud=115200):
        self.who_i_am = who_i_am
        self.address_format = address_format
        self.baud = baud
        self.property_lock = None
        self.updated = False

        self.has_new_command = False
        self.command_packet = ""

        self.__setattr__ = self.setattr

    def setattr(self, key, value):
        self.property_lock.acquire()
        object.__setattr__(self, key, value)
        self.property_lock.release()

    def _receive(self, packet):
        self.receive(packet)

    def receive(self, packet):
        pass

    def send(self, packet):
        self.has_new_command = True
        self.command_packet = packet

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

        if self.log_data:
            self.logger.start_time()

    def _start(self):
        for robot_port in self.ports.values():
            robot_port.start()

    def _stop(self):
        for robot_port in self.ports.values():
            robot_port.stop()

    def main(self):
        pass

    def close(self):
        pass

    def run(self):
        self._start()
        try:
            while True:
                self.main()
        except:
            traceback.print_exc()
        finally:
            self.close()
            self._stop()
