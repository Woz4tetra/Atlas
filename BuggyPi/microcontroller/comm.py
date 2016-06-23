"""
Written by Ben Warwick

comm.py, written for RoboQuasar3.0
Version 3/6/2016
=========

Handles direct serial communications with the micro-controller.

This class is a subclass of threading.Thread. It's internally used by
data.py. It continually updates the passed in SensorPool object with
what ever sensor data comes in.

This library follows a home-baked serial packet protocol. data.py handles
all data conversion.
"""

import glob
import sys
import threading
import time
import traceback

import serial

from microcontroller.logger import Logger


class Communicator(threading.Thread):
    # Flag used to kill all running threads (mainly the serial thread in
    # Communicator)
    exit_flag = False

    def __init__(self, sensors_pool, baud_rate=115200, log_data=True,
                 log_name=None, log_dir=None, handshake=True):
        """
        Constructor for Communicator

        :param baud_rate: Data communication rate. Make sure it matches your
            device's baud rate! (For MicroPython it doesn't matter)
        :param sensors_pool: The SensorPool object in which to put the sensor
            data
        :param use_handshake: For Arduino boards or any micro-controller that
            reboots when a program connects over serial.
        :return: Communicator object
        """
        self.serial_ref, self.address = self.find_port(baud_rate)

        if handshake:
            self.initialized = self.handshake()
        else:
            self.initialized = False

        self.sensor_pool = sensors_pool
        self.buffer = ""

        self.time0 = time.time()
        self.thread_time = 0

        self.log_data = log_data
        if self.log_data and self.initialized:
            self.log = Logger(log_name, log_dir)

        super(Communicator, self).__init__()

    def record(self, name, value=None, **values):
        if value is not None or len(values) == 0:
            self.log.enq(name, value)
        elif value is None and len(values) > 0:
            self.log.enq(name, values)

    def run(self):
        """
        Runs and constantly, updates packets for different sensors

        see "self.log.enq(sensor.name, sensor._properties.copy())"
            super important to copy()! sensor._properties is passed by reference
            otherwise and may change before being recorded.
        :return: None
        """
        try:
            while not Communicator.exit_flag:
                self.thread_time = round(time.time() - self.time0)
                if self.serial_ref.inWaiting() > 0:
                    packets, status = self.read_packets()
                    if status is False:
                        print("Serial read failed...")
                        raise KeyboardInterrupt
                    else:
                        for packet in packets:
                            if len(packet) > 0:
                                sensor = self.sensor_pool.update(packet)
                                if sensor is not None and self.log_data:
                                    self.log.enq(
                                        sensor.name, sensor._properties.copy())
                                else:
                                    if "Traceback" in packet:
                                        print("MicroPython ", end="")
                                    print("-- ", packet)
                    if self.log_data:
                        self.log.record()
                time.sleep(0.0005)
        except KeyboardInterrupt:
            traceback.print_exc()

        if self.log_data:
            self.log.close()
        self.serial_ref.close()

    def read_packets(self):
        try:
            incoming = self.serial_ref.read(self.serial_ref.inWaiting())
            self.buffer += incoming.decode('ascii')
            packets = self.buffer.split('\r\n')

            if self.buffer[-2:] != '\r\n':
                self.buffer = packets.pop(-1)
            else:
                self.buffer = ""
            return packets, True
        except:
            traceback.print_exc()
            return [], False

    def write_byte(self, data):
        try:
            self.serial_ref.write(data)
        except:
            print("Serial write failed...")
            traceback.print_exc()
            self.stop()

    def put(self, packet):
        """
        Directly puts a string into serial. This was revised from having a
        command queue (the queue would fill faster than it would empty).
        For use in data.py by Command objects.

        :param packet: A string formed by a Command object
        :return: None
        """
        self.write_byte(bytearray(packet, 'ascii'))

    def handshake(self):
        """
        Ensures communication between serial and Arduino (or any
        micro-controller that needs it, MicroPython does not)

        :return: None
        """
        print("Waiting for ready flag from %s..." % self.address)

        self.put("R")

        start_time = time.time()

        read_flag = None
        try:
            while read_flag != "buggypi":
                read_flag = self.serial_ref.readline().decode("ascii").strip(
                    "\r\n")
                print(read_flag)
                time.sleep(0.0005)
                if time.time() - start_time > 1:
                    start_time = time.time()
                    self.put('R')
        except:
            traceback.print_exc()
            self.stop()
            return False

        print("Ready flag received!")
        return True

    def find_port(self, baud_rate):
        """
        Tries all possible addresses as found by _possibleAddresses() until
        a serial connection is established.

        :param baud_rate: The baud rate of the serial connection
        :return: serial.Serial - instance of the serial object
        """
        address = None
        serial_ref = None
        for possible_address in self.possible_addrs():
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
        """
        Returns all addresses that could be a micro-controller.
        TODO: Figure out how to implement multiple board support

        :return: A list of strings containing all likely addresses
        """
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
        Sets the exit_flag to True. Stops the serial read thread

        :return: None
        """
        print("")
        Communicator.exit_flag = True
