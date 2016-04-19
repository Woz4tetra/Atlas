
import pyb
from objects import *

class Communicator(object):
    def __init__(self, command_pool):
        self.serial_ref = pyb.USB_VCP()

        self.command_pool = command_pool

        self.packet = ""
        self.should_reset = False

    def write_packet(self, sensor):
        self.serial_ref.write(sensor.get_packet())

    def read_command(self):
        if self.serial_ref.any():
            character = self.serial_ref.read().decode("ascii")

            # an unreasonably sized packet could indicate something has gone wrong
            while character != '\r' and len(self.packet) < 1024:
                if character == "R":
                    self.should_reset = True
                elif character == "H":
                    self.should_reset = True
                    self.serial_ref.write("R")
                else:
                    self.packet += character
                character = self.serial_ref.read().decode("ascii")

            if type(self.packet) == str:
                self.command_pool.update(self.packet)
                self.packet = ""

    def close(self):
        self.serial_ref.close()
