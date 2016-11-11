import pyb
from math import pi


class LidarTurret():
    def __init__(self, uart_bus, ticks_per_rotation=38, baud_rate=115200):
        self.uart = pyb.UART(uart_bus, baud_rate, read_buf_len=255)
        self.ticks_per_rotation = ticks_per_rotation
        
        self.counts = 0
        self.angle = 0.0
        self.rotations = 0
        self.distance = 0
        self.rotated = False
    
    def read(self):
        if self.uart.any():
            data = self.uart.readline().decode('ascii')[:-1].split('\t')
            
            if len(data) == 2:
                if data[0].isdigit():
                    self.counts = int(data[0])
                    self.angle = self.counts / self.ticks_per_rotation * 2 * pi
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
            
