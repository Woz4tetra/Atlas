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
import struct
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

    def __init__(self, baud_rate, sensors_pool, use_handshake=True,
                 file_name=None, directory=None, log_data=True):
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

        if use_handshake:
            self.handshake()

        self.sensor_pool = sensors_pool

        self.time0 = time.time()
        self.thread_time = 0

        self.buffer = ""

        self.log_data = log_data
        if self.log_data:
            self.log = Logger(file_name, directory)

        super(Communicator, self).__init__()

    def run(self):
        """
        Runs and constantly, updates packets for different sensors

        :return: None
        """

        try:
            while not Communicator.exit_flag:
                self.thread_time = round(time.time() - self.time0)
                if self.serial_ref.inWaiting() > 0:
                    incoming = self.serial_ref.read(self.serial_ref.inWaiting())
                    self.buffer += incoming.decode('ascii')
                    packets = self.buffer.split('\r')

                    if self.buffer[-1] != '\r':
                        self.buffer = packets.pop(-1)
                    else:
                        self.buffer = ""
                    for packet in packets:
                        if len(packet) > 0:
                            sensor = self.sensor_pool.update(packet)

                            if sensor is not None:
                                if self.log_data:
                                    self.log.enq(sensor.name,
                                                 sensor._properties)
                            else:
                                packet = packet.replace("\r", "")
                                packet = packet.replace("\n", "")
                                if "Traceback" in packet:
                                    print("MicroPython ", end="")
                                if ">>> " in packet:
                                    self.stop()
                                print(packet)
                if self.log_data:
                    self.log.record()
                time.sleep(0.0005)
        except:
            traceback.print_exc()

        if self.log_data:
            self.log.close()
        self.serial_ref.close()

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
        print("Waiting for ready flag from %s..." % self.address)

        read_flag = None

##        self.serial_ref.flushInput()
##        self.serial_ref.flushOutput()

        counter = 0
        self.serial_ref.write(b"R\r")

        try:
            while read_flag != "R":
##                time.sleep(0.01)
                read_flag = self.serial_ref.read().decode("ascii")
                print(read_flag, end="")
                time.sleep(0.001)

                if counter % 1000 == 0:
                    self.serial_ref.write(b'\x03')  # control-c to stop pyboard
                    time.sleep(0.01)
                    self.serial_ref.write(b'\x03')
                    time.sleep(0.25)
                    self.serial_ref.write(struct.pack("B", 4))
                    time.sleep(0.25)
                    self.serial_ref.write(b"R\r")
                    time.sleep(0.25)
                counter += 1
        except:
            self.stop()
        
        # self.serial_ref.write("\r")
        self.serial_ref.flushInput()
        self.serial_ref.flushOutput()

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
        Communicator.exit_flag = True

    def dump(self):
        """
        Dump all contents of serial

        :return: None
        """
        self.serial_ref.flushInput()
        self.serial_ref.flushOutput()
