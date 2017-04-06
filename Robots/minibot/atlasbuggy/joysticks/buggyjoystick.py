import os
import time
import math

import pygame
from atlasbuggy import get_platform


try:
    with os.popen('stty size', 'r') as terminal_window:
        terminal_rows, terminal_cols = (int(x) for x in
                                        terminal_window.read().split())
except ValueError:
    terminal_rows = terminal_cols = -1


class JoysticksNotFoundError(Exception):
    """No joysticks found"""


class BuggyJoystick:
    """
    A generic joystick class using pygame. This class captures any joystick events
    """

    def __init__(self, axes_mapping, axes_dead_zones, button_mapping):
        """
        :param axes_mapping: A list of axis names that correspond to the axis number pygame assigns
        :param axes_dead_zones: If the corresponding axis number is less than a value in this list, it is considered zero
        :param button_mapping: A list of button names that correspond to the button number pygame assigns
        """
        platform = get_platform()
        if platform != "mac":
            os.environ["SDL_VIDEODRIVER"] = "dummy"
        pygame.init()
        pygame.joystick.init()

        # search for all available joysticks and initialize them
        joysticks = [pygame.joystick.Joystick(x) for x in
                     range(pygame.joystick.get_count())]
        if len(joysticks) == 0:
            raise JoysticksNotFoundError("No joysticks found!")

        for joy in joysticks:
            joy.init()
            # print(joy.get_name(), joy.get_id(), joy.get_init(),
            #       joy.get_numaxes())

        self.axis_to_name = axes_mapping
        self.button_to_name = button_mapping

        # create dictionaries that maps names to button/axis IDs
        self.name_to_axis = self.create_mapping(axes_mapping)
        self.name_to_button = self.create_mapping(button_mapping)

        self.dead_zones = [abs(value) for value in axes_dead_zones]
        self.axis_flipped = [math.copysign(1, value) for value in axes_dead_zones]

        # current values
        self.axes = [0.0] * len(axes_mapping)
        self.buttons = [False] * len(button_mapping)
        self.dpad = (0, 0)

        self.axis_changed = [False] * len(axes_mapping)
        self.prev_buttons = [False] * len(button_mapping)
        self.prev_dpad = (0, 0)

        self.t0 = time.time()
        self.active = False

        super(BuggyJoystick, self).__init__()

    @staticmethod
    def create_mapping(list_mapping):
        """
        Turn a list of strings into a dictionary that maps the strings
        to an index in the list

        :param list_mapping: a list of parameter names
        """
        dict_mapping = {}
        for index, name in enumerate(list_mapping):
            if bool(name) != False:
                dict_mapping[name] = index
        return dict_mapping

    def update(self):
        """
        Go through every queued pygame event.
        If an event is JOYAXISMOTION, assign to the axes property list
        If an event is JOYHATMOTION, assign to the dpad tuple
        If an event is JOYBUTTONDOWN, assign True to the buttons property list
        If an event is JOYBUTTONUp, assign False to the buttons property list

        If an axis or button isn't encapsulated in the list, raise an error.

        :return: False if pygame signals the QUIT event
        """
        # Go through every event pygame sees
        for event in pygame.event.get():
            self.t0 = time.time()
            # if event.type != pygame.NOEVENT:
                # print(event)
            if event.type == pygame.QUIT:
                return "exit"

            # if an axis event occurred
            if event.type == pygame.JOYAXISMOTION:
                self.active = True
                # set value to the axis if it is outside the deadzone
                # else set it to zero
                if event.axis < len(self.axes):
                    if abs(event.value) > self.dead_zones[event.axis]:
                        value = self.axis_flipped[event.axis] * event.value
                    else:
                        value = 0.0

                    # if the value changed
                    if self.axes[event.axis] != value:
                        # update the current value
                        self.axes[event.axis] = value
                        self.axis_changed[event.axis] = True
                else:
                    raise ValueError("Unregistered axis! '%s'. Please add "
                                     "it to your joystick class." % event.axis)

            # if the dpad updated, set the current value and call the
            # dpad callback
            elif event.type == pygame.JOYHATMOTION:
                self.active = True
                self.dpad = event.value

            # if a button was pressed, set it to True and call the button
            # down callback
            elif event.type == pygame.JOYBUTTONDOWN:
                self.active = True
                if event.button < len(self.buttons):
                    self.buttons[event.button] = True
                else:
                    raise ValueError(
                        "Unregistered button! '%s'. Please add "
                        "it to your joystick class." % event.button)

            # if a button was released, set it to False and call the button
            # up callback
            elif event.type == pygame.JOYBUTTONUP:
                self.active = True
                if event.button < len(self.buttons):
                    self.buttons[event.button] = False
                else:
                    raise ValueError(
                        "Unregistered button! '%s'. Please add "
                        "it to your joystick class." % event.button)
        # TODO: find better solution. Keeps listening to bumpers
        # if self.active and (time.time() - self.t0) > 1:  # if no events received for more than timeout, signal stop
            # return "error"

    def get_button(self, name):
        """
        Get the value of a button using the name as reference
        :param name: The name of a button listed in button_mapping in __init__
        :return: The value of the button (bool)
        """
        return self.buttons[self.name_to_button[name]]

    def button_updated(self, name):
        """
        Check if the button updated its value
        :param name: The name of a button listed in button_mapping in __init__
        :return: bool
        """
        prev_button = self.prev_buttons[self.name_to_button[name]]
        button = self.buttons[self.name_to_button[name]]
        if prev_button != button:
            self.prev_buttons[self.name_to_button[name]] = button
            return True
        else:
            return False

    def get_axis(self, name):
        """
        Get the value of an axis using the name as reference
        :param name: The name of an axis listed in axis_mapping in __init__
        :return: The value of the axis (float, 0.0...1.0)
        """
        return self.axes[self.name_to_axis[name]]

    def axis_updated(self, name):
        """
        Check if the axis updated its value
        :param name: The name of an axis listed in axis_mapping in __init__
        :return: bool
        """
        if self.axis_changed[self.name_to_axis[name]]:
            self.axis_changed[self.name_to_axis[name]] = False
            return True
        else:
            return False

    def dpad_updated(self):
        if self.prev_dpad != self.dpad:
            self.prev_dpad = self.dpad
            return True
        else:
            self.prev_dpad = self.dpad
            return False


    @staticmethod
    def fill_line(line):
        """
        Fill the rest of the line with spaces according to the terminal window size
        :param line: a string
        :return: a string filled with spaces
        """
        if terminal_cols != -1:
            return line + (" " * (terminal_cols - len(line)))
        else:
            return line

    def __str__(self):
        """Format the current values of the joystick nicely"""
        string = self.fill_line("axes:") + "\n"
        counter = 0
        line = ""
        for name, index in self.name_to_axis.items():
            line += "%s: %5.2f    " % (name, self.axes[index])

            if counter % 3 == 0 and index != 0:
                string += self.fill_line(line) + "\n"
                line = ""
            counter += 1

        string += self.fill_line(line) + "\n"

        string += self.fill_line("buttons:") + "\n"
        counter = 0
        line = ""
        for name, index in self.name_to_button.items():
            line += "%s: %i    " % (name, self.buttons[index])
            if counter % 3 == 0 and index != 0:
                string += self.fill_line(line) + "\n"
                line = ""
            counter += 1

        string += self.fill_line(line) + "\n"

        string += "dpad:" + self.fill_line(str(self.dpad)) + "\n"

        lines = len(string.split("\n")) + 1

        return string + ("\033[F" * lines)
