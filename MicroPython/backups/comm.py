
import pyb
from objects import *

class Communicator(object):
    def __init__(self, *commands):
        self.serial_ref = pyb.USB_VCP()

        self.command_pool = CommandPool(commands)

        self.packet = ""
        self.reset_code = None

    def write_packet(self, sensor):
        self.serial_ref.write(sensor.get_packet())

    def read_command(self):
        if self.serial_ref.any():
            # character = self.serial_ref.read()
            # if character is not None:
            #     character = character.decode("ascii")
            #
            # while character != '\r':
            #     if character == "R" or character == "H":
            #         pyb.LED(4).toggle()
            #         self.should_reset = True
            #         if character == "H":
            #             self.serial_ref.write("R")
            #         return
            #     if character is not None:
            #         self.packet += character
            #     character = self.serial_ref.read()
            #     if character is not None:
            #         character = character.decode("ascii")
            #
            # del character
            self.packet = self.serial_ref.readline().decode("ascii")
            if "R" in self.packet or "H" in self.packet:
                pyb.LED(2).toggle()
                self.reset_code = "R"
                if "H" in self.packet:
                    self.reset_code = "H"
                    self.serial_ref.write("R\r")
                return

            if type(self.packet) == str:
                self.command_pool.update(self.packet)
                self.packet = ""


    def close(self):
        self.serial_ref.close()
