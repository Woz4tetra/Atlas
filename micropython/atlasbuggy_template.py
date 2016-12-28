import pyb
import time
import sys

class RobotInterface:
    def __init__(self):
        self.serial_ref = pyb.USB_VCP()
        self.buffer = b''
        self.paused = True
        self.who_i_am = ""  # put who_i_am ID here
        self.packet_end = "\n"

    def read_packets(self):
        incoming = self.serial_ref.read(self.serial_ref.any())
        self.buffer = incoming.decode('ascii')
        packets = self.buffer.split(self.packet_end)

        if len(self.buffer) > len(self.packet_end):
            self.buffer = packets.pop(-1)
        
        return packets

    def write(self, command):
        command += self.packet_end
        self.serial_ref.write(command.encode('ascii'))

    def ready(self):
        self.write("ready!")
        self.paused = False
        
        # reset sensors
    
    def read_commands(self):
        if self.serial_ref.any():
            for packet in read_packets():
                if packet == "ready?":
                    self.ready()
                elif packet == "whoareyou":
                    self.write("iam%s\t" % (who_i_am, ...))  # fill with any addition data 
                elif packet == "stop":
                    self.paused = True
                elif len(packet) > 0:
                    pass  # fill with custom commands
    
    def write_sensors(self):
        if not self.paused:
            self.write("" % ...)  # fill with sensor data to write
    
    def run(self):
        while True:
            self.write_sensors()
            
            self.read_commands()
            
    # ----- custom methods -----

RobotInterface().run()
