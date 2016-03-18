import time
import math
from PID import *


class PIDTester (object):
    def __init__(self):
        self.buggy = Buggy()
        self.PID = PID(self.buggy.pos, self.buggy.angle)

    def change_angle(self, direction):
        print self.buggy.angle
        while self.PID.value != direction:
            print self.buggy.angle, "old angle"
            new_angle = self.PID.update(self.buggy.angle, direction)
            print new_angle, "new_angle"
            direction = if self.buggy.angle > new_angle:
                            "left"
                        else: "right" 
            self.buggy.turn(direction, abs(new_angle - self.buggy.angle))
            

class Buggy(object):
    def __init__ (self, pos = (10.0,10.0), angle = 0.0):
        self.pos = pos
        self.angle = angle
        self.max_change = math.pi*(5/180)
        self.speed = 0.1

    def turn(self, direction, amount):
        if abs(amount) > self.max_change:
            raise Exception "tried to turn too far"
        if direction == "right":
            self.angle -= (self.angle - amount) % 360
        else if direction == "left":
            self.angle += (self.angle + amount) % 360
        else:
            raise Exception "Not valid turn command"

    def forward(self):
        self.pos[0] += self.speed * math.cos(self.angle)
        self.pos[1] += self.speed * math.sin(self.angle)
