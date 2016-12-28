"""
Handles direct serial communications with the micro-controller.

Using multi-threading, it continuously listens to data from a microcontroller
via a serial connection and passes it to a SensorPool object for parsing.

This class only supports one microcontroller at a time (i.e. a one-to-one
connection)
"""

import glob
import sys
import threading
import time
import traceback

import serial

from atlasbuggy.microcontroller.logger import Logger


class Communicator(threading.Thread):
    # Flag used to kill the serial thread
    exit_flag = False

    def __init__(self, sensors_pool, address=None, exclude_addresses=None,
                 baud_rate=115200, log_data=True,
                 log_name=None, log_dir=None, handshake=True):
        # if no address is provided, search for possible candidates
        if address is None:
            self.serial_ref, self.address = self.find_port(
                baud_rate, exclude_addresses)
        else:
            self.address = address
            self.serial_ref = serial.Serial(port=self.address,
                                            baudrate=baud_rate,
                                            timeout=1)
        self.start_time = time.time()

        self.sensor_pool = sensors_pool

        # Where serial dumps EVERYTHING that it currently sees
        # It is parsed at the thread's most convenient time
        self.buffer = ""

        # a timer to make sure the thread is running (used in is_alive())
        self.start_time = time.time()
        self.thread_time = 0
        
        # tell the microcontroller that we're starting
        if handshake:
            self.initialized = self.handshake()
        else:
            self.initialized = False

        # initialize the data logger
        self.log_data = log_data
        if self.log_data and self.initialized and len(self.sensor_pool) > 0:
            self.log = Logger(log_name, log_dir)

        super(Communicator, self).__init__()

    def record(self, name, value=None, **values):
        """
        A wrapper for Logger's record method. Record any data with this
        method
        """
        if self.log_data:
            if value is not None or len(values) == 0:
                self.log.enq(name, value)
            elif value is None and len(values) > 0:
                self.log.enq(name, values)

    def run(self):
        """
        Constantly parses packets and hands the data to the corresponding
        sensors

        bug fix: "self.log.enq(sensor.name, sensor._properties.copy())":
            super important to copy()! sensor._properties is passed by reference
            otherwise and may change before being recorded.
        """
        try:
            if self.initialized:
                while not Communicator.exit_flag:
                    self.thread_time = round(time.time() - self.start_time)
                    if self.serial_ref.inWaiting() > 0:
                        packets, status = self.read_packets()
                        if status is False:
                            print("Serial read failed...")
                            raise KeyboardInterrupt
                        else:
                            self.parse_packets(packets)
                        if self.log_data:
                            self.log.record()
                    time.sleep(0.0005)
        except KeyboardInterrupt:
            traceback.print_exc()
        finally:
            # weird bug: can't send any commands after while loop exits
            if self.log_data:
                self.log.close()
            self.serial_ref.close()

    def is_alive(self):
        """A way to check if serial is hanging the thread"""
        current_time = int(time.time() - self.start_time)
        status = abs(current_time - self.thread_time) < 2
        if not status:
            print("Communicator thread stopped responding!!! Thread was out of "
                  "sync by %s seconds" % (current_time - self.thread_time))
        return status
        
    def enable_callbacks(self):
        self.sensor_pool.enable_callbacks = True
    
    def disable_callbacks(self):
        self.sensor_pool.enable_callbacks = False

    def parse_packets(self, packets):
        """
        Give all parsed packets to the correct sensors and log them if
        log_data is True
        """
        for packet in packets:
            if len(packet) > 0:
                sensor = self.sensor_pool.update(packet)
                if sensor is not None:
                    if self.log_data:
                        # have to copy properties or else it might be
                        # changed before the logger can record it!!
                        # (sensor._properties is a reference not the
                        # dictionary itself)
                        self.log.enq(sensor.name, sensor._properties.copy())
                else:
                    if "Traceback" in packet:
                        print("MicroPython ", end="")
                    print("-- ", packet)

    def read_packets(self):
        """
        Read all available data on serial and split them into packets as
        indicated by the characters \r\n. If serial crashes, the method
        returns False indicating that the communicator thread should be stopped.
        """
        try:
            if self.serial_ref.inWaiting() > 0:
                incoming = self.serial_ref.read(self.serial_ref.inWaiting())
                self.buffer += incoming.decode('ascii')
                packets = self.buffer.split('\r\n')

                if self.buffer[-2:] != '\r\n':
                    self.buffer = packets.pop(-1)
                else:
                    self.buffer = ""
                return packets, True
            else:
                return [], True
        except:
            traceback.print_exc()
            return [], False

    def write_byte(self, data):
        """Safely write a byte over serial"""
        if not Communicator.exit_flag:
            try:
                self.serial_ref.write(data)
            except:
                print("Serial write failed...")
                # traceback.print_exc()
                self.stop()

    def put(self, packet):
        """
        Directly puts a string into serial.
        For use in data.py by Command objects.

        packet should be a string
        """
        self.write_byte(bytearray(packet, 'ascii'))

    def handshake(self):
        """Signals to the microcontroller that the program is ready"""
        print("Waiting for ready flag from %s..." % self.address)

        return self.send_signal('ready?', 'ready!')

    def close_comm(self):
        # self.send_signal('stop', 'stopping')  # causes really bad exceptions
        self.put('stop\r\n')
        self.stop()

    def send_signal(self, send_signal, recv_signal):
        send_signal += '\r\n'
        self.put(send_signal)

        start_time = time.time()

        read_flag = None
        try:
            while read_flag != recv_signal:
                try:
                    read_flag = self.serial_ref.readline().decode(
                        "ascii").strip("\r\n")
                except UnicodeDecodeError:
                    print("invalid character")
                print(read_flag)
                time.sleep(0.0005)
                if time.time() - start_time > 0.005:
                    start_time = time.time()
                    self.put(send_signal)
        except:
            traceback.print_exc()
            self.stop()
            return False

        return True

    def find_port(self, baud_rate, exclude_addresses):
        """
        Tries all possible addresses as found by possible_addrs() until
        a serial connection is established.
        """
        address = None
        serial_ref = None
        for possible_address in self.possible_addrs():
            if exclude_addresses is not None:
                skip_address = False
                for excluded_address in exclude_addresses:
                    # if * is present, then only search up to that index
                    if "*" in excluded_address:
                        end_index = excluded_address.find("*")

                        # if end indices don't match, addresses aren't the same
                        if len(possible_address) >= end_index:
                            # compare truncated addresses
                            if possible_address[0:end_index] == \
                                    excluded_address[0:end_index]:
                                skip_address = True
                    elif excluded_address == possible_address:
                        skip_address = True
                if skip_address:
                    print("skipping:", possible_address)
                    continue
            try:
                serial_ref = serial.Serial(port=possible_address,
                                           baudrate=baud_rate,
                                           timeout=0.005)
                address = possible_address
                break
            except serial.SerialException:
                pass
        if address is None:
            raise Exception(
                "No boards could be found! Did you plug it in? Try "
                "entering the address manually.")
        else:
            return serial_ref, address

    @staticmethod
    def possible_addrs():
        """Returns all addresses that could be a microcontroller."""
        if sys.platform.startswith('darwin'):  # OS X
            return glob.glob('/dev/tty.usbmodem[0-9]*') + \
                   glob.glob('/dev/cu.usbmodem[0-9]*')

        elif (sys.platform.startswith('linux') or
                  sys.platform.startswith('cygwin')):  # linux
            return ['/dev/ttyACM' + str(i) for i in range(10)]

        elif sys.platform.startswith('win'):  # Windows
            return ['COM' + str(i + 1) for i in range(256)]

        else:
            raise EnvironmentError('Unsupported platform')

    def stop(self):
        """
        Sets the exit_flag to True indicating that the communication thread
        should be stopped
        """
        if not self.is_alive():
            print("Communicator thread wasn't running. Nothing to stop.")
        Communicator.exit_flag = True
