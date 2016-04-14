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
from dotable import Dotable

sys.path.insert(0, "../")
import config


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

        platform = config.get_platform()
        if platform == "mac":
            self.update = self.update_mac
        elif platform == "linux":
            self.update = self.update_linux
        elif platform == "win":
            self.update = self.update_mac
        else:
            raise EnvironmentError("Hey... how did you get here?\n"
                                   "You should've failed in config...")
        super(BuggyJoystick, self).__init__()

    def run(self):
        while not BuggyJoystick.exit_flag:
            self.update()
            time.sleep(0.001)

    @staticmethod
    def stop():
        BuggyJoystick.exit_flag = True

    def update_mac(self):
        pass

    def update_linux(self):
        pass

    def updateButtons(self, event, value):
        pass


class GCJoystick(BuggyJoystick):
    def __init__(self):
        super(GCJoystick, self).__init__()

        self.deadzoneStick = 0.15

        self.mainStick = Dotable({
            'x': 0,
            'y': 0
        })
        self.cStick = Dotable({
            'x': 0,
            'y': 0
        })
        self.triggers = Dotable({
            'L': 0,
            'R': 0
        })

        self.buttons = Dotable({
            "A": False,
            "B": False,
            "X": False,
            "Y": False,
            "Z": False,
            "L": False,
            "R": False,
            "start": False,
        })

        self.dpad = Dotable({
            "left": False,
            "right": False,
            "up": False,
            "down": False,
        })

    def update_mac(self):
        event = pygame.event.poll()
        # if event.type != pygame.NOEVENT:
        #     print(event)
        if event.type == pygame.QUIT:
            self.done = True

        if event.type == pygame.JOYAXISMOTION:
            if event.axis == 0:
                self.mainStick.x = event.value
            elif event.axis == 1:
                self.mainStick.y = event.value
            elif event.axis == 2:
                self.cStick.x = event.value
            elif event.axis == 3:
                self.cStick.y = event.value
            elif event.axis == 4:
                self.triggers.L = event.value
            elif event.axis == 5:
                self.triggers.R = event.value

            if (abs(self.mainStick.x) < self.deadzoneStick and
                        abs(self.mainStick.y) < self.deadzoneStick):
                self.mainStick.x = 0
                self.mainStick.y = 0
            if (abs(self.cStick.x) < self.deadzoneStick and
                        abs(self.cStick.y) < self.deadzoneStick):
                self.cStick.x = 0
                self.cStick.y = 0
        elif event.type == pygame.JOYBUTTONDOWN:
            self.updateButtons(event, True)

        elif event.type == pygame.JOYBUTTONUP:
            self.updateButtons(event, False)

    def update_linux(self):
        event = pygame.event.poll()
        if event.type == pygame.QUIT:
            self.done = True

        if event.type == pygame.JOYAXISMOTION:
            if event.axis == 0:
                self.mainStick.x = event.value
            elif event.axis == 1:
                self.mainStick.y = -event.value
            elif event.axis == 2:
                self.cStick.y = -event.value
            elif event.axis == 3:
                self.triggers.L = event.value
            elif event.axis == 4:
                self.triggers.R = event.value
            elif event.axis == 5:
                self.cStick.x = event.value

            if (abs(self.mainStick.x) < self.deadzoneStick and
                        abs(self.mainStick.y) < self.deadzoneStick):
                self.mainStick.x = 0
                self.mainStick.y = 0
            if (abs(self.cStick.x) < self.deadzoneStick and
                        abs(self.cStick.y) < self.deadzoneStick):
                self.cStick.x = 0
                self.cStick.y = 0
        elif event.type == pygame.JOYBUTTONDOWN:
            self.updateButtons(event, True)

        elif event.type == pygame.JOYBUTTONUP:
            self.updateButtons(event, False)

    def updateButtons(self, event, value):
        if event.button == 0:
            self.buttons.X = value
        elif event.button == 1:
            self.buttons.A = value
        elif event.button == 2:
            self.buttons.B = value
        elif event.button == 3:
            self.buttons.Y = value
        elif event.button == 4:
            self.buttons.L = value
        elif event.button == 5:
            self.buttons.R = value
        elif event.button == 7:
            self.buttons.Z = value
        elif event.button == 9:
            self.buttons.start = value
        elif event.button == 12:
            self.dpad.up = value
        elif event.button == 13:
            self.dpad.right = value
        elif event.button == 14:
            self.dpad.down = value
        elif event.button == 15:
            self.dpad.left = value

    def __str__(self):
        return "x: %s, y: %s\n" \
               "cx: %s, cy: %s\n" \
               "A: %s, B: %s, X: %s, Y: %s\n" \
               "start: %s, Z: %s\n" \
               "L t: %s, R t: %s\n" \
               "left: %s, right: %s, up: %s, down: %s\n" \
               "L: %s, R: %s" % (
                   self.mainStick.x, self.mainStick.y,
                   self.cStick.x, self.cStick.y,
                   self.buttons.A, self.buttons.B, self.buttons.X,
                   self.buttons.Y, self.buttons.start, self.buttons.Z,
                   self.buttons.L, self.buttons.R,
                   self.dpad.left, self.dpad.right, self.dpad.up,
                   self.dpad.down,
                   self.triggers.L, self.triggers.R)


class WiiUJoystick(BuggyJoystick):
    def __init__(self):
        super(WiiUJoystick, self).__init__()

        self.deadzoneStick = 0.2

        self.leftStick = Dotable({
            'x': 0,
            'y': 0
        })
        self.rightStick = Dotable({
            'x': 0,
            'y': 0
        })

        self.buttons = Dotable({
            "A": False,
            "B": False,
            "X": False,
            "Y": False,
            "ZR": False,
            "ZL": False,
            "R": False,
            "L": False,
            "plus": False,
            "minus": False,
            "Ljoy": False,
            "Rjoy": False,
        })

        self.dpad = Dotable({
            "left": False,
            "right": False,
            "up": False,
            "down": False,
        })

    def update_mac(self):
        raise NotImplementedError()

    def update_linux(self):
        event = pygame.event.poll()
        if event.type == pygame.QUIT:
            self.done = True

        if event.type == pygame.JOYAXISMOTION:
            if event.axis == 0:
                self.leftStick.x = event.value
            elif event.axis == 1:
                self.leftStick.y = -event.value
            elif event.axis == 2:
                self.rightStick.y = event.value
            elif event.axis == 3:
                self.rightStick.x = event.value

            if (abs(self.leftStick.x) < self.deadzoneStick and
                        abs(self.leftStick.y) < self.deadzoneStick):
                self.leftStick.x = 0
                self.leftStick.y = 0
            if (abs(self.rightStick.x) < self.deadzoneStick and
                        abs(self.rightStick.y) < self.deadzoneStick):
                self.rightStick.x = 0
                self.rightStick.y = 0

        if event.type == pygame.JOYHATMOTION:
            if event.value[0] == 1 and event.value[1] == 0:
                self.dpad.left = False
                self.dpad.right = True
                self.dpad.up = False
                self.dpad.down = False
            elif event.value[0] == 1 and event.value[1] == 1:
                self.dpad.left = False
                self.dpad.right = True
                self.dpad.up = True
                self.dpad.down = False
            elif event.value[0] == 0 and event.value[1] == 1:
                self.dpad.left = False
                self.dpad.right = False
                self.dpad.up = True
                self.dpad.down = False
            elif event.value[0] == -1 and event.value[1] == 1:
                self.dpad.left = True
                self.dpad.right = False
                self.dpad.up = True
                self.dpad.down = False
            elif event.value[0] == -1 and event.value[1] == 0:
                self.dpad.left = True
                self.dpad.right = False
                self.dpad.up = False
                self.dpad.down = False
            elif event.value[0] == -1 and event.value[1] == -1:
                self.dpad.left = True
                self.dpad.right = False
                self.dpad.up = False
                self.dpad.down = True
            elif event.value[0] == 0 and event.value[1] == -1:
                self.dpad.left = False
                self.dpad.right = False
                self.dpad.up = False
                self.dpad.down = True
            elif event.value[0] == 1 and event.value[1] == -1:
                self.dpad.left = False
                self.dpad.right = True
                self.dpad.up = False
                self.dpad.down = True
            else:
                self.dpad.left = False
                self.dpad.right = False
                self.dpad.up = False
                self.dpad.down = False

        elif event.type == pygame.JOYBUTTONDOWN:
            self.updateButtons(event, True)

        elif event.type == pygame.JOYBUTTONUP:
            self.updateButtons(event, False)

            # if event.type != pygame.NOEVENT:
            #     print(event)

    def updateButtons(self, event, value):
        if event.button == 0:
            self.buttons.Y = value
        elif event.button == 1:
            self.buttons.B = value
        elif event.button == 2:
            self.buttons.A = value
        elif event.button == 3:
            self.buttons.X = value
        elif event.button == 4:
            self.buttons.L = value
        elif event.button == 5:
            self.buttons.R = value
        elif event.button == 6:
            self.buttons.ZL = value
        elif event.button == 7:
            self.buttons.ZR = value
        elif event.button == 8:
            self.buttons.minus = value
        elif event.button == 9:
            self.buttons.plus = value
        elif event.button == 10:
            self.buttons.Ljoy = value
        elif event.button == 11:
            self.buttons.Rjoy = value
        elif event.button == 12:
            self.dpad.up = value
        elif event.button == 13:
            self.dpad.right = value
        elif event.button == 14:
            self.dpad.down = value
        elif event.button == 15:
            self.dpad.left = value

    def __str__(self):
        return "rx: %s, ry: %s\n" \
               "lx: %s, ly: %s\n" \
               "A: %s, B: %s, X: %s, Y: %s\n" \
               "+: %s, -: %s\n" \
               "L: %s, R: %s\n" \
               "ZL: %s, ZR: %s\n" \
               "Ljoy: %s, Rjoy: %s\n" \
               "left: %s, right: %s, up: %s, down: %s\n" % (
                   self.rightStick.x, self.rightStick.y,
                   self.leftStick.x, self.leftStick.y,
                   self.buttons.A, self.buttons.B, self.buttons.X,
                   self.buttons.Y, self.buttons.plus, self.buttons.minus,
                   self.buttons.L, self.buttons.R,
                   self.buttons.ZL, self.buttons.ZR,
                   self.buttons.Ljoy, self.buttons.Rjoy,
                   self.dpad.left, self.dpad.right, self.dpad.up,
                   self.dpad.down)


def joystick_init(joy_type="gc"):
    pygame.init()
    pygame.display.init()
    # screen = pygame.display.set_mode((200, 200))
    pygame.joystick.init()

    if joy_type == "gc":
        joystick = GCJoystick()
    elif joy_type == "wiiu":
        joystick = WiiUJoystick()
    else:
        raise ValueError("Please supply valid joystick type: " + str(joy_type))
    joystick.start()
    return joystick


if __name__ == '__main__':
    if len(sys.argv) > 0:
        joystick_type = sys.argv[1]
    else:
        joystick_type = "gc"

    joystick = joystick_init(joy_type=joystick_type)
    try:
        while not joystick.done:
            print(joystick)

            time.sleep(0.005)
    except KeyboardInterrupt:
        joystick.stop()
