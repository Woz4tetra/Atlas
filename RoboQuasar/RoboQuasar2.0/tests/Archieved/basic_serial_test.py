

import sys
import os
import glob
import serial

class Communicator():
    def __init__(self, baud_rate):
        self.serialRef = self._findPort(baud_rate)
        self._handshake()

    def _handshake(self):
        print("Waiting for ready flag...")
        
        read_flag = self.serialRef.read()
        
        time.sleep(0.5)
        while read_flag != 'R':
            print(read_flag, end="")
            read_flag = self.serialRef.read()
        
        self.serialRef.write("\n")
        self.serialRef.flushInput()
        self.serialRef.flushOutput()
        print("Arduino initialized!")
    
    def _findPort(self, baud_rate):
        address = None
        serial_ref = None
        for possible_address in self._possibleAddresses():
            try:
                serial_ref = serial.Serial(port=possible_address,
                                           baudrate=baud_rate)
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
import time

comm = Communicator(115200)

servo_val = "00"
led_val = True

while True:
    packet = "00\t02\t" + servo_val + "\n"
    comm.serialRef.write(packet)
    time.sleep(0.25)
    print("sent:", repr(packet))
    print("recv:", repr(comm.serialRef.readline()))
    
    packet = "01\t01\t" + str(int(led_val)) + "\n"
    comm.serialRef.write(packet)
    time.sleep(0.25)
    print("sent:", repr(packet))
    print("recv:", repr(comm.serialRef.readline()))
    
    if servo_val == "00":
        servo_val = "9c"
    else:
        servo_val = "00"
    led_val = not led_val

