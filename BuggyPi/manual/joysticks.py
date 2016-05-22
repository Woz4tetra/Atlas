
from manual.buggy_joystick import *


class GCJoystick(BuggyJoystick):
    def __init__(self):
        super(GCJoystick, self).__init__()

        self.deadzoneStick = 0.15

        self.main_stick = {
            'x': 0,
            'y': 0
        }
        self.c_stick = {
            'x': 0,
            'y': 0
        }
        self.triggers = {
            'L': 0,
            'R': 0
        }

        self.buttons = {
            "A": False,
            "B": False,
            "X": False,
            "Y": False,
            "Z": False,
            "L": False,
            "R": False,
            "start": False,
        }

        self.dpad = {
            "left": False,
            "right": False,
            "up": False,
            "down": False,
        }

    def update(self):
        event = pygame.event.poll()
        # if event.type != pygame.NOEVENT:
        #     print(event)
        if event.type == pygame.QUIT:
            self.done = True

        if event.type == pygame.JOYAXISMOTION:
            if event.axis == 0:
                self.main_stick.x = event.value
            elif event.axis == 1:
                self.main_stick.y = event.value
            elif event.axis == 2:
                self.c_stick.x = event.value
            elif event.axis == 3:
                self.c_stick.y = event.value
            elif event.axis == 4:
                self.triggers.L = event.value
            elif event.axis == 5:
                self.triggers.R = event.value

            if (abs(self.main_stick.x) < self.deadzoneStick and
                        abs(self.main_stick.y) < self.deadzoneStick):
                self.main_stick.x = 0
                self.main_stick.y = 0
            if (abs(self.c_stick.x) < self.deadzoneStick and
                        abs(self.c_stick.y) < self.deadzoneStick):
                self.c_stick.x = 0
                self.c_stick.y = 0
        elif event.type == pygame.JOYBUTTONDOWN:
            self.update_buttons(event, True)

        elif event.type == pygame.JOYBUTTONUP:
            self.update_buttons(event, False)

    def update_buttons(self, event, value):
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
                   self.main_stick['x'], self.main_stick['y'],
                   self.c_stick['x'], self.c_stick['y'],
                   self.buttons['A'], self.buttons['B'], self.buttons['X'],
                   self.buttons['Y'], self.buttons['start'], self.buttons['Z'],
                   self.buttons['L'], self.buttons['R'],
                   self.dpad['left'], self.dpad['right'], self.dpad['up'],
                   self.dpad['down'],
                   self.triggers['L'], self.triggers['R'])


class WiiUJoystick(BuggyJoystick):
    def __init__(self):
        super(WiiUJoystick, self).__init__()

        self.deadzoneStick = 0.2

        self.leftStick = {
            'x': 0,
            'y': 0
        }
        self.rightStick = {
            'x': 0,
            'y': 0
        }

        self.buttons = {
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
        }

        self.dpad = {
            "left": False,
            "right": False,
            "up": False,
            "down": False,
        }

    def update(self):
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
            self.update_dpad(event)

        elif event.type == pygame.JOYBUTTONDOWN:
            self.update_buttons(event, True)

        elif event.type == pygame.JOYBUTTONUP:
            self.update_buttons(event, False)

    def update_dpad(self, event):
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

    def update_buttons(self, event, value):
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
                   self.rightStick['x'], self.rightStick['y'],
                   self.leftStick['x'], self.leftStick['y'],
                   self.buttons['A'], self.buttons['B'], self.buttons['X'],
                   self.buttons['Y'], self.buttons['plus'], self.buttons['minus'],
                   self.buttons['L'], self.buttons['R'],
                   self.buttons['ZL'], self.buttons['ZR'],
                   self.buttons['Ljoy'], self.buttons['Rjoy'],
                   self.dpad['left'], self.dpad['right'], self.dpad['up'],
                   self.dpad['down'])


if __name__ == '__main__':
    if len(sys.argv) > 0:
        joystick_type = sys.argv[1]
    else:
        joystick_type = "gc"

    if joystick_type == "gc":
        joystick = joystick_init(GCJoystick)
    elif joystick_type == "wiiu":
        joystick = joystick_init(WiiUJoystick)
    else:
        raise ValueError("Invalid joystick type:", joystick_type)
    try:
        while not joystick.done:
            print(joystick)
            time.sleep(0.005)
    except KeyboardInterrupt:
        joystick.stop()
