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

        self.done = False

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


def joystick_init(joy_class):
    pygame.init()
    pygame.display.init()
    # screen = pygame.display.set_mode((200, 200))
    pygame.joystick.init()

    joystick_ref = joy_class()
    joystick_ref.start()
    return joystick_ref
