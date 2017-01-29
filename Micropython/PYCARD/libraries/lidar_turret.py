import pyb
from math import pi


class LidarTurret():
    def __init__(self, uart_bus, baud_rate=115200):
        self.uart = pyb.UART(uart_bus, baud_rate, read_buf_len=65536)
        
        self.counts = 0
        self.rotations = 0
        self.distance = 0
        
        self.start = True
        self.direction = True
        self.speed = 255
        
        self.buffer = ''
        
    def parse_packets(self):
        self.buffer += self.uart.read(self.uart.any()).decode('ascii')
        print(self.buffer)
        packets = self.buffer.split("\n")
        
        if self.buffer[-1] != "\n":
            self.buffer += packets.pop(-1)
            
        return packets
    
    def read(self):
        if self.uart.any():
#            data = self.uart.readline()[:-1].decode('ascii').split("\t")
            for packet in self.parse_packets():
                data = packet.split("\t")
        
                if len(data) == 2:
                    if data[0].isdigit():
                        self.counts = int(data[0])
                    if data[1].isdigit():
                        self.rotations = int(data[1])
                    return True
                
                elif len(data) == 1:
                    if data[0].isdigit():
                        self.distance = int(data[0])
                    return True
        return False
    
    def received(self):
        return self.read()
    
    def send_command(self, command_type, value=None):
        command = command_type.encode('ascii')
        if value is not None:
            command += str(value).encode('ascii')
        command += b'\n'
        
        self.uart.write(command)
    
    def send_start(self):
        if not self.start:
            self.send_command("B")
            self.start = True
    
    def send_stop(self):
        if self.start:
            self.send_command("E")
            self.start = False
    
    def change_direction(self, direction=None):
        if direction is None:
            self.direction = not self.direction
        else:
            self.direction = direction
        
        self.send_command("D", int(self.direction))
    
    def change_speed(self, speed):
        self.speed = speed
        self.send_command("M", speed % 0x100)
        
            
