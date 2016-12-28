import glob
import time
import traceback
from queue import Queue
from threading import Thread, Lock
import serial
import serial.tools.list_ports
from pprint import pprint

from atlasbuggy.logs import Logger


class RobotSerialPort(Thread):
    def __init__(self, port_info, baud, packet_end, logger, log_data,
                 debug_prints):
        self.address = port_info.device  # directory to open (in /dev)
        self.serial_ref = None  # pyserial Serial class reference, assigned when connection is made
        self.port_info = port_info

        self.debug_prints = debug_prints

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

        # from address_format, generate possible addresses
        # attempt to open the serial port
        self.serial_ref = \
            serial.Serial(port=self.address, baudrate=baud)

        # Find the ID of this port. We didn't check this while opening the port
        # because the correct ports will be matched up to the correct
        # RobotObject later
        self.find_who_i_am()

        self.start_time = time.time()
        self.thread_time = 0

        self.packet_parse_successful = True

        super(RobotSerialPort, self).__init__()

    def run(self):
        try:
            while not self.should_stop:
                self.thread_time = int(time.time() - self.start_time)

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
                        try:
                            self.robot_object.receive(packet)
                        except:
                            traceback.print_exc()
                            print("Received packets:")
                            pprint(packets)
                            print("\nError packet:", repr(packet))

                            print("'receive' function for '%s' raised an error."
                                  % self.who_i_am)

                            self.packet_parse_successful = False
                            break

                        if self.log_data and self.logger.time0 != 0:
                            self.logger.record(self.who_i_am, packet)

                    if not self.packet_parse_successful:
                        break

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

        if self.debug_prints:
            print("'%s' while loop exited" % self.who_i_am)
        self.stop()

    def is_running(self):
        if self.should_stop:
            print("should_stop was", self.should_stop)
            return False

        current_time = int(time.time() - self.start_time)
        status = abs(current_time - self.thread_time) < 2
        if not status:
            print("Communicator thread stopped responding!!! Thread was out of "
                  "sync by %s seconds" % (current_time - self.thread_time))
        return status

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
            print("Exception occurred during serial read.")
            print("Possible causes:\n"
                  "   The device isn't running code\n"
                  "   The device isn't functioning\n"
                  "   Packets aren't parsing correctly "
                  "(packet_end's don't match)\n"
                  "   Non-ascii characters are being sent")
            traceback.print_exc()

            self.stop()
            return [], False

    def stop(self):
        # send stop packet. Don't wait for feedback. There's some weird race
        # condition going on
        if not self.should_stop:
            time.sleep(0.05)
            self.write_packet("stop\n")

            if self.debug_prints:
                print("Sent stop flag")
            time.sleep(0.05)

            self.should_stop = True

            self.serial_ref.close()

            if self.debug_prints:
                print("Closing serial")

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
                traceback.print_exc()
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
        if self.debug_prints:
            for packet in packets:
                print("> %s" % repr(packet))

    def find_who_i_am(self):
        time.sleep(0.005)
        self.serial_ref.flush()
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

        if self.debug_prints:
            print(self.address, "is ready")
        else:
            print(self.address, "is ready ", end="")

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

        if self.debug_prints:
            print("%s has ID '%s'" % (self.address, self.who_i_am))
        else:
            print("and has ID '%s'" % self.who_i_am)


class RobotObject:
    def __init__(self, who_i_am, address_format=None, baud=115200):
        self.who_i_am = who_i_am
        self.address_format = address_format
        self.baud = baud
        self.property_lock = None
        self.updated = False

        self.command_packets = Queue(maxsize=255)

        # to prevent two threads accessing data at the same time, overwrite
        # set to include property_lock. This is a hacky way to avoid having to
        # set all properties through an arbitrary set function
        self.__setattr__ = self.setattr

    def setattr(self, key, value):
        self.property_lock.acquire()
        object.__setattr__(self, key, value)
        self.property_lock.release()

    def receive_first(self, packet):
        """
        initialize any data here. If the who_am_i packet contains data,
        it's passed here
        :param packet: The first packet received by the robot object's port
        :return:
        """
        pass

    def receive(self, packet):
        """
        parse incoming packets received by the corresponding port
        :param packet: A packet (string) received from the robot object's port
        :return:
        """
        pass

    def send(self, packet):
        """
        queue the new packet for sending
        :param packet: A packet (string) received from the robot object's port
        :return:
        """
        self.command_packets.put(packet)

    def did_update(self):
        if self.updated:
            self.updated = False
            return True
        else:
            return False


class RobotInterface:
    def __init__(self, *robot_objects, joystick=None, baud_rate=115200,
                 packet_end='\n', log_data=True, log_name=None, log_dir=None,
                 debug_prints=False):
        self.debug_prints = debug_prints

        self.log_data = log_data
        if self.log_data:
            # if no directory provided, use the ":today" flag (follows the
            # project.py directory reference convention)
            if log_dir is None:
                log_dir = ":today"
            self.logger = Logger(log_name, log_dir)
        else:
            self.logger = None

        self.joystick = joystick

        # open all available ports
        self.ports = {}
        self.objects = {}
        for robot_object in robot_objects:
            self.objects[robot_object.who_i_am] = robot_object

        for port_info in serial.tools.list_ports.comports():
            if port_info.serial_number is not None:
                port = RobotSerialPort(port_info, baud_rate,
                                       packet_end, self.logger, log_data,
                                       self.debug_prints)
                self.ports[port.who_i_am] = port

        if len(self.ports) < len(self.objects):
            self.print_port_info()
            time.sleep(0.01)  # wait to finish printing before raising error

            self._stop()

            raise ValueError(
                "Not enough ports to match provided robot objects... "
                "Are all devices properly connected?")

        # assign ports based on identity
        for who_i_am in self.objects.keys():
            if who_i_am not in self.ports.keys():
                self.print_port_info()
                time.sleep(0.01)  # wait to finish printing before raising error

                self._stop()

                raise ValueError("RobotObject with ID '%s' not found among the "
                                 "discovered ports... Are they all named "
                                 "correctly?" % who_i_am)

            self.ports[who_i_am].robot_object = self.objects[who_i_am]

            # ensures properties being set by robot objects aren't being
            # accessed by the port at the same time
            self.objects[who_i_am].property_lock = \
                self.ports[who_i_am].property_lock

            # send the first packet received to the object
            try:
                if len(self.ports[who_i_am].first_packet) > 0:
                    self.objects[who_i_am].receive_first(
                        self.ports[who_i_am].first_packet
                    )
                elif self.debug_prints:
                    print("'%s' no data found accompanying who_i_am packet")
            except:
                self._stop()
                traceback.print_exc()
                raise ValueError(
                    "receive_first function for '%s' raised an error." %
                    who_i_am
                )

    def print_port_info(self):
        print("\nDiscovered ports:")
        for port in self.ports.values():
            print("\tPort ID: '%s'" % port.who_i_am)
            pprint(port.port_info.__dict__)
        print()

    def _start(self):
        for robot_port in self.ports.values():
            robot_port.start()

        if self.log_data:
            self.logger.start_time()

    def _stop(self):
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

    def are_threads_running(self):
        for robot_port in self.ports.values():
            if not robot_port.is_running():
                print("Port with ID '%s' signaled stop" %
                      robot_port.who_i_am)
                return False
        return True

    def run(self):
        """
        Call this method to start the robot and to start receiving data
        :return:
        """
        self._start()

        try:
            while self.are_threads_running():
                if self.loop() is False:
                    if self.debug_prints:
                        print("loop signaled to exit")
                    break

                if self.joystick is not None:
                    if self.joystick.update() is False:
                        break

        except:
            traceback.print_exc()
        finally:
            if self.debug_prints:
                print("Calling close function")
            self.close()
            if self.debug_prints:
                print("Calling stop function")
            self._stop()
