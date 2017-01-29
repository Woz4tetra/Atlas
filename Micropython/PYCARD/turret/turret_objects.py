from math import pi

import pyb
import time

from data import *
from libraries.lidar_turret import LidarTurret

class LidarSensor(Sensor):
    def __init__(self, sensor_id, uart_bus):
        super(LidarSensor, self).__init__(sensor_id, ['i8', 'i64', 'u16'])
        self.lidar = LidarTurret(uart_bus)
    
    def recved_data(self):
        return self.lidar.received()
    
    def update_data(self):
        return self.lidar.counts, self.lidar.rotations, self.lidar.distance
        
    def reset(self):
        self.lidar.send_start()
    
    def stop(self):
        self.lidar.send_stop()
    
    def __str__(self):
        return "c: %5.0i, r: %5.0i, d: %5.0i" % (self.lidar.counts,
            self.lidar.rotations, self.lidar.distance) 
        

class BlueLEDcommand(Command):
    def __init__(self, command_id, initial_state=0):
        super().__init__(command_id, 'u8')
        self.led = pyb.LED(4)
        self.set_state(initial_state)

    def set_state(self, state):
        self.led.intensity(state)

    def callback(self, state):
        self.set_state(state)

    def reset(self):
        self.set_state(0)
