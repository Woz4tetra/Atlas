import serial
import glob
from threading import Thread, Lock
import traceback
import time


class RobotSerialPort(Thread):
    opened_addresses = []

    def __init__(self, robot_object):
        self.robot_object = robot_object

        self.address = None
        self.serial_ref = None

        self.who_am_i = -1
        self.who_am_i_header = "iam"

        self.property_lock = Lock()

        self.address_format = self.robot_object.address_format
        if type(self.address_format) == str:
            self.address_format = [self.address_format]

        for address_format in self.address_format:
            for address in glob.glob(address_format):
                if address not in RobotSerialPort.opened_addresses:
                    try:
                        self.serial_ref = \
                            serial.Serial(port=address,
                                          baudrate=self.robot_object.baud,
                                          timeout=0.005)
                    except serial.SerialException:
                        pass
                    self.address = address
                    RobotSerialPort.opened_addresses.append(self.address)
                    break

        if self.address is None:
            raise ConnectionError(
                "Could not find serial port... %s" %
                self.robot_object.address_format)
        print("Using", self.address)

        self.buffer = ""
        self.should_stop = False
        self.start_time = time.time()
        self.thread_time = 0

        self.find_who_i_am()

        super(RobotSerialPort, self).__init__()

    def run(self):
        try:
            while not self.should_stop:
                self.thread_time = round(time.time() - self.start_time)
                if self.serial_ref.is_open and self.serial_ref.in_waiting > 0:
                    packets, status = self.read_packets()
                    if not status:
                        raise Exception("Packet read failed...")

                    self.property_lock.acquire()
                    if len(packets) > 0:
                        self.robot_object.updated = True
                    for packet in packets:
                        self.robot_object.parse_packet(packet)
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
                packets = self.buffer.split('\n')

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

    def parse_who_am_i_packet(self, packet):
        if packet[0:len(self.who_am_i_header)] == self.who_am_i_header:
            # the rest of the packet is the ID number
            return int(packet[len(self.who_am_i_header):])
        else:
            return -1

    def find_who_i_am(self):
        time.sleep(0.005)
        time0 = time.time()
        self.write_packet("whoareyou\n")

        while self.who_am_i == -1:
            packets, status = self.read_packets()
            if not status:
                raise Exception("Serial read failed... Couldn't get board ID")

            for packet in packets:
                self.who_am_i = self.parse_who_am_i_packet(packet)
                if self.who_am_i != -1:
                    break
            time1 = time.time()
            if time1 - time0 > 1:
                self.write_packet("whoareyou\n")
            time.sleep(0.005)


class RobotObject:
    def __init__(self, who_am_i: int, address_format, baud=115200):
        self.who_am_i = who_am_i
        self.address_format = address_format
        self.baud = baud
        self.property_lock = None
        self.updated = False

        self.__setattr__ = self.setattr

    def setattr(self, key, value):
        self.property_lock.acquire()
        object.__setattr__(self, key, value)
        self.property_lock.release()

    def parse_packet(self, packets):
        pass

    def did_update(self):
        if self.updated:
            self.updated = False
            return True
        else:
            return False


class RobotInterface:
    def __init__(self, *robot_objects):
        self.ports = {}
        # self.objects = {}
        for robot_object in robot_objects:
            self.ports[robot_object.who_am_i] = \
                RobotSerialPort(robot_object)
            robot_object.property_lock = \
                self.ports[robot_object.who_am_i].property_lock
            # self.objects[robot_object.name] = robot_object

    def _start(self):
        for robot_port in self.ports.values():
            robot_port.start()

    def _stop(self):
        for robot_port in self.ports.values():
            robot_port.stop()

    def main(self):
        pass

    def run(self):
        self._start()
        try:
            while True:
                self.main()
        except:
            self._stop()