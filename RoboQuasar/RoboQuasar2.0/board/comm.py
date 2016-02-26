"""
    Written by Ben Warwick

    comm.py, written for RoboQuasar1.0
    Version 12/7/2015
    =========

    Handles direct serial communications with the arduino.

    This class is a subclass of threading.Thread. It constantly updates the
    SensorData (constructor parameter) object with new data received from
    serial. It will also send any commands put on the CommandQueue (also a
    constructor parameter).

    This library follows a home-baked serial packet protocol. data.py handles
    all data conversion. Please refer to objects.py for proper usage tips and
    data.py for details of the sensor and command packet protocol.
"""

import serial
import time
import os
import sys
import glob
import threading


class Communicator(threading.Thread):
    # Flag used to kill all running threads (mainly the serial thread in
    # Communicator)
    exit_flag = False

    def __init__(self, baud_rate, sensors_pool,
                 use_handshake=True):
        self.serialRef = self._findPort(baud_rate)
        self.serialRef.flushInput()
        self.serialRef.flushOutput()

        if use_handshake:
            self._handshake()

        self.sensor_pool = sensors_pool

        self.start_time = time.time()
        self.thread_time = 0

        super(Communicator, self).__init__()

    def run(self):
        """
        Runs and constantly, updates packets for different sensors, puts
        command packets on command queue into serial.

        :return: None
        """

        while self.exit_flag == False:
            self.thread_time = round(time.time() - self.start_time)
            packet = bytearray()
            incoming = self.serialRef.read()
            while incoming != b'\r':
                if incoming != None and incoming != b'':
                    packet += incoming
                incoming = self.serialRef.read()
            if len(packet) > 0:
                self.sensor_pool.update(packet)

    def put(self, packet):
        self.serialRef.write(bytearray(packet, 'ascii'))

    def _handshake(self):
        """
        Ensures communication between serial and arduino (or any microcontroller
        that needs it, micropython does not)

        :return: None
        """
        read_flag = self.serialRef.read()

        print("Waiting for ready flag...")
        time.sleep(0.5)
        while read_flag != 'R':
            print(read_flag, end="")
            read_flag = self.serialRef.read()

        self.serialRef.write("\r")
        self.serialRef.flushInput()
        self.serialRef.flushOutput()
        print("Board initialized!")

    def _findPort(self, baud_rate):
        """
        Tries all possible addresses as found by _possibleAddresses() until
        a serial connection is established.

        :param baud_rate: The baud rate of the serial connection
        :return: serial.Serial - instance of the serial object
        """
        address = None
        serial_ref = None
        for possible_address in self._possibleAddresses():
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
    def _possibleAddresses():
        """
        An internal method used by _initSerial to search all possible
        USB serial addresses.

        :return: A list of strings containing all likely addresses
        """
        if sys.platform.startswith('darwin'):  # OS X
            devices = os.listdir("/dev/")
            arduino_devices = []
            for device in devices:
                if device.find("cu") > -1 or \
                        device.find("tty") > -1:
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
        Communicator.exit_flag = True

    def dump(self):
        self.serialRef.flushInput()
        self.serialRef.flushOutput()
