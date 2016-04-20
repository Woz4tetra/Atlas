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

import serial
import time
import os
import sys
import glob
import threading
import struct


class Communicator(threading.Thread):
    # Flag used to kill all running threads (mainly the serial thread in
    # Communicator)
    exit_flag = False

    def __init__(self, baud_rate, sensors_pool, use_handshake=True):
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
        self.serial_ref = self.find_port(baud_rate)

        if use_handshake:
            self.handshake()

        self.sensor_pool = sensors_pool

        self.time0 = time.time()
        self.thread_time = 0

        super(Communicator, self).__init__()

    def run(self):
        """
        Runs and constantly, updates packets for different sensors

        :return: None
        """

        invalid_packets = 0
        while not Communicator.exit_flag:
            self.thread_time = round(time.time() - self.time0)
            if self.serial_ref.inWaiting() > 0:
                packet = bytearray()
                incoming = self.serial_ref.read()
                while incoming != b'\r':
                    if incoming is not None and incoming != b'':
                        packet += incoming
                    incoming = self.serial_ref.read()
                if len(packet) > 0:
                    invalid_packets = self.sensor_pool.update(packet)

                    if invalid_packets > 512:
                        Communicator.exit_flag = True

    def put(self, packet):
        """
        Directly puts a string into serial. This was revised from having a
        command queue (the queue would fill faster than it would empty).
        For use in data.py by Command objects.

        :param packet: A string formed by a Command object
        :return: None
        """
        self.serial_ref.write(bytearray(packet, 'ascii'))

    def handshake(self):
        """
        Ensures communication between serial and Arduino (or any
        micro-controller that needs it, MicroPython does not)

        :return: None
        """
        print("Waiting for ready flag...")

        read_flag = None

        self.serial_ref.reset_input_buffer()
        self.serial_ref.reset_output_buffer()

        def write_resets():
            self.serial_ref.write(struct.pack("B", 4))
            self.serial_ref.write(b"\r")

            self.serial_ref.write(b"H")
            self.serial_ref.write(b"\r")

        counter = 0
        write_resets()

        print("\n---------")
        while read_flag != "R":
            time.sleep(0.001)
            read_flag = self.serial_ref.read().decode("ascii")
            print(read_flag, end="")
            time.sleep(0.001)

            counter += 1
            if counter % 50 == 0:
                write_resets()
                

        print("\n---------")

        # self.serial_ref.write("\r")
        self.serial_ref.reset_input_buffer()
        self.serial_ref.reset_output_buffer()
        print("\nBoard initialized!")

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
            except:
                pass
        if address is None:
            raise Exception(
                    "No boards could be found! Did you plug it in? Try "
                    "entering the address manually.")
        else:
            return serial_ref

    @staticmethod
    def possible_addrs():
        """
        Returns all addresses that could be a micro-controller.
        TODO: Figure out how to implement multiple board support

        :return: A list of strings containing all likely addresses
        """
        if sys.platform.startswith('darwin'):  # OS X
            devices = os.listdir("/dev/")
            arduino_devices = []
            for device in devices:
                if device.find("cu.usbmodem") > -1 or \
                        device.find("tty.usbmodem") > -1:
                    arduino_devices.append("/dev/" + device)
            return arduino_devices

        elif (sys.platform.startswith('linux') or
                  sys.platform.startswith('cygwin')):  # linux

            return glob.glob('/dev/tty[A-Za-z]*')

        elif sys.platform.startswith('win'):  # Windows
            return ['COM' + str(i + 1) for i in range(256)]

        else:
            raise EnvironmentError('Unsupported platform')

    def stop(self):
        """
        Sets the exit_flag to True. Stops the serial read thread

        :return: None
        """
        Communicator.exit_flag = True

    def dump(self):
        """
        Dump all contents of serial

        :return: None
        """
        self.serial_ref.flushInput()
        self.serial_ref.flushOutput()
