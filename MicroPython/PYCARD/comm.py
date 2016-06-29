
import pyb
from objects import *

class Communicator(object):
    def __init__(self, *commands, uart_bus=None):
        if uart_bus is not None:
            self.using_usb = False
            self.serial_ref = pyb.UART(uart_bus, 115200, read_buf_len=1000,
                                       timeout_char=10000)
            print("Initialized! Ready to go!")
        else:
            self.using_usb = True
            self.serial_ref = pyb.USB_VCP()

        self.command_pool = CommandPool(commands)

        self.reset = None

        self.buffer = ""

    def write_packet(self, sensor):
        packet = sensor.get_packet()
##        print(packet)
        self.serial_ref.write(sensor.get_packet())
        pyb.delay(1)

    def read_command(self):
        if self.serial_ref.any():
            for packet in self.read_packets():
                if "ready?" in packet:
                    pyb.LED(2).on()
                    self.reset = True
                    self.serial_ref.write("ready!\r\n")
                else:
                    self.command_pool.update(packet)
                    
    def read_packets(self):
        incoming = self.serial_ref.read(self.serial_ref.any())
        self.buffer += incoming.decode('ascii')
        packets = self.buffer.split("\r\n")

        if self.buffer[-2:] != "\r\n":
            self.buffer = packets.pop(-1)
        else:
            self.buffer = ""
        return packets 
    
    def close(self):
        self.serial_ref.close()
