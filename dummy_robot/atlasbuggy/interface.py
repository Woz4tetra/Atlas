import time
import struct
import traceback
from multiprocessing import Process, Lock, Queue, Event
import serial
import serial.tools.list_ports
from serial.serialutil import SerialException
import pprint

from atlasbuggy.logs import Logger, Parser
from atlasbuggy.errors import *


class RobotObject:
    def __init__(self, whoiam):
        """
        A container for data received from the corresponding microcontroller.

        Make sure the whoiam ID corresponds to the one defined on the microcontroller
        (see templates for details).

        Define object variables here

        :param whoiam:
        """
        self.whoiam = whoiam

        self.command_packets = Queue(maxsize=255)

    def receive_first(self, packet):
        """
        Overwrite this method when subclassing RobotObject if you're expecting initial data

        Initialize any data defined in __init__ here.
        If the who_am_i packet contains data, it's passed here. Otherwise, this method isn't called

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

        Queue a new packet for sending. The packet end (\n) will automatically be appended

        :param packet: A packet (string) to send to the microcontroller without the packet end character
        :return: None
        """
        self.command_packets.put(packet)


class RobotInterface:
    def __init__(self, *robot_objects, joystick=None,
                 log_data=True, log_name=None, log_dir=None,
                 debug_prints=False, loop_updates_per_second=60, port_updates_per_second=720):
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

            while self._are_ports_active():
                if not self._dequeue_packets():
                    break

                if not self._main_loop():
                    break

                self._send_commands()

                if self.joystick is not None:
                    if self.joystick.update() is False:
                        break

                if not self.clock.update() and not self.lag_warning_thrown:
                    print("Warning. Main loop is running slow.")
                    self.lag_warning_thrown = True

        except KeyboardInterrupt:
            pass

        self._close_all()

    def packet_received(self, timestamp, whoiam):
        """
        Overwrite this method

        A callback function for every time a packet is received

        :param timestamp: The time the packet arrived
        :param whoiam: Which robot object the packet went to
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
                    if self.packet_received(timestamp, whoiam) is False:
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


class RobotInterfaceSimulator:
    def __init__(self, file_name, directory, start_index=0, end_index=-1, *robot_objects):
        self.objects = {}
        for robot_object in robot_objects:
            self.objects[robot_object.whoiam] = robot_object

        self.parser = Parser(file_name, directory, start_index, end_index)

        self.current_index = 0

    def packet_received(self, timestamp, whoiam):
        return True

    def run(self):
        for index, timestamp, whoiam, packet in self.parser:
            if timestamp == -1:
                self.objects[whoiam].receive_first(packet)
            else:
                self.objects[whoiam].receive(packet)
                if not self.packet_received(timestamp, whoiam):
                    print("packet_received signalled to exit")
                    break

            self.current_index = index

        self.close()

    def close(self):
        pass


class Clock:
    def __init__(self, loops_per_second):
        self.loop_time = 0
        if loops_per_second is not None:
            self.seconds_per_loop = 1 / loops_per_second
        else:
            self.seconds_per_loop = None

        self.current_time = 0
        self.time_diff = 0
        self.offset = 0

        self.on_time = True

    def start(self):
        self.loop_time = time.time()
        self.current_time = time.time()

    def update(self):
        if self.seconds_per_loop is None:
            return True

        self.current_time = time.time()
        if self.current_time != self.loop_time:
            self.time_diff = self.current_time - self.loop_time
            self.offset = self.seconds_per_loop - self.time_diff

            if self.offset > 0:
                self.on_time = True
                time.sleep(self.offset)
            else:
                self.on_time = False

            self.loop_time = time.time()

            return self.offset > 0
        else:
            return False


class RobotSerialPort(Process):
    """
    A multi-threaded wrapper for an instance of pyserial Serial.
    A RobotInterface class manages all instances of this class.
    This class is for internal use only
    """

    def __init__(self, port_info, debug_prints, queue, lock, updates_per_second):
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
        super(RobotSerialPort, self).__init__(target=self.update, args=(queue, lock))

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

    def update(self, queue, lock):
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
                            queue.put((self.whoiam, time.time() - self.start_time, packet))

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
