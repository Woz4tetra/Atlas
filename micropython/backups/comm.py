import pyb

from data import *


class Communicator(object):
    def __init__(self, *commands, uart_bus=None, baud=115200):
        if uart_bus is not None:
            self.using_usb = False
            self.serial_ref = pyb.UART(uart_bus, baud, read_buf_len=1000,
                                       timeout_char=10000)
            print("Initialized! Ready to go!")
        else:
            self.using_usb = True
            self.serial_ref = pyb.USB_VCP()

        self.command_pool = CommandPool(commands)

        self.reset = False
        self.stop = False

        self.buffer = ""

    def write_packet(self, sensor):
        self.serial_ref.write(sensor.get_packet())
        pyb.delay(1)

    def signal_stop(self):
        print("Stop signal received")
        self.stop = True
        self.reset = False
        self.serial_ref.write("stopping\r\n")

    def signal_reset(self):
        print("Reset signal received")
        self.reset = True
        self.stop = False
        self.serial_ref.write("ready!\r\n")

    def read_command(self):
        if self.serial_ref.any():
            for packet in self.read_packets():
                if "ready?" == packet:
                    self.signal_reset()
                elif "stop" == packet:
                    self.signal_stop()
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

    def should_reset(self):
        if self.reset:
            self.reset = False
            return True
        else:
            return False

    def should_stop(self):
        if self.stop:
            self.stop = False
            return True
        else:
            return False

    def close(self):
        self.serial_ref.close()
