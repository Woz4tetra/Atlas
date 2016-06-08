"""
Written by Ben Warwick

joystick.py, written for RoboQuasar3.0
Version 4/5/2015
=========

Allows for out-of-the-box interface with a Wii U Pro or gamecube controller
(connected with the corresponding mayflash adapter)

"""

import sys
import threading
import time

import pygame

sys.path.insert(0, "../")

class BuggyJoystick(threading.Thread):
    exit_flag = False

    # TODO: add multiple joystick support
    def __init__(self):
        self.deadzoneStick = 0.0

        self.values_changed = False

        joysticks = [pygame.joystick.Joystick(x) for x in
                     range(pygame.joystick.get_count())]
        assert len(joysticks) > 0
        for joy in joysticks:
            joy.init()
            print(joy.get_name(), joy.get_id(), joy.get_init(),
                  joy.get_numaxes())

        super(BuggyJoystick, self).__init__()

    def run(self):
        while not BuggyJoystick.exit_flag:
            self.update()
            time.sleep(0.001)

    @staticmethod
    def stop():
        BuggyJoystick.exit_flag = True

    def update(self):
        pass

    def update_buttons(self, event, value):
        pass


class TestJoystick(BuggyJoystick):
    def __init__(self):
        super(TestJoystick, self).__init__()
        self.prev_event = None
        self.event = None
        
    def update(self):
        event = pygame.event.poll()
        if event.type != pygame.NOEVENT:
            self.prev_event = self.event
            if event.type != pygame.JOYAXISMOTION:
                self.event = event
            
        elif event.type == pygame.QUIT:
            self.stop()

def joystick_init(joy_class):
    pygame.init()
    pygame.display.init()
    # screen = pygame.display.set_mode((200, 200))
    pygame.joystick.init()

    joystick_ref = joy_class()
    joystick_ref.start()
    return joystick_ref

if __name__ == '__main__':
    pygame.init()
    pygame.display.init()
    pygame.joystick.init()
    
    joystick = TestJoystick()
    while True:
        joystick.update()
        if joystick.prev_event is not None:# and joystick.prev_event.button != joystick.event.button: #not almost_equal(joystick.prev_event.value, joystick.event.value):
            print(joystick.event)

        time.sleep(0.01)
