
import pyb
from objects import *

class Communicator(object):
    def __init__(self, *commands):
        self.serial_ref = pyb.USB_VCP()

        self.command_pool = CommandPool(commands)

        self.packet = ""
        self.reset = None

    def write_packet(self, sensor):
        self.serial_ref.write(sensor.get_packet())

    def read_command(self):
        if self.serial_ref.any():
            self.packet = self.serial_ref.readline().decode("ascii")
            if "R" in self.packet:
                pyb.LED(2).toggle()
                self.reset = True
                self.serial_ref.write("R\r")
                return

            if type(self.packet) == str:
                self.command_pool.update(self.packet)
                self.packet = ""


    def close(self):
        self.serial_ref.close()
