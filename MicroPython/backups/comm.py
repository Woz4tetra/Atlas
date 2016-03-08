
import pyb
from objects import *

class Communicator(object):
    def __init__(self, sensor_queue, command_pool):
        self.serial_ref = pyb.USB_VCP()

        self.sensor_queue = sensor_queue
        self.command_pool = command_pool

    def write_packet(self):
        # packet = self.sensor_queue.get()
        # self.serial_ref.write(packet)
        # print("\r\npacket:", packet)
        self.serial_ref.write(self.sensor_queue.get())

    def read_command(self):
        if self.serial_ref.any():
            packet = self.serial_ref.readline().decode("ascii")

            if type(packet) == str:
                self.command_pool.update(packet)

            del packet
            # packet = bytes()
            # incoming = self.serial_ref.read()
            # start_time = pyb.millis()
            # while incoming != b'\r' (pyb.millis() - start_time) <= 0.005:
            #     if incoming != None and incoming != b'':
            #         packet += incoming
            #     incoming = self.serial_ref.read()
            # if len(packet) > 0:
            #     self.command_pool.update(packet)

    def close(self):
        self.serial_ref.close()
