import pygame
import os
from atlasbuggy import project

try:
    with os.popen('stty size', 'r') as terminal_window:
        terminal_rows, terminal_cols = (int(x) for x in
                                        terminal_window.read().split())
except ValueError:
    terminal_rows = terminal_cols = -1


class BuggyJoystick:
    def __init__(self, axes_mapping, axes_dead_zones, button_mapping):
        platform = project.get_platform()
        if platform != "mac":
            os.environ["SDL_VIDEODRIVER"] = "dummy"
        pygame.init()
        pygame.joystick.init()

        # search for all available joysticks and initialize them
        joysticks = [pygame.joystick.Joystick(x) for x in
                     range(pygame.joystick.get_count())]
        assert len(joysticks) > 0
        for joy in joysticks:
            joy.init()
            # print(joy.get_name(), joy.get_id(), joy.get_init(),
            #       joy.get_numaxes())

        self.axis_to_name = axes_mapping
        self.button_to_name = button_mapping

        # create dictionaries that maps names to button/axis IDs
        self.name_to_axis = self.create_mapping(axes_mapping)
        self.name_to_button = self.create_mapping(button_mapping)

        self.dead_zones = axes_dead_zones

        # current values
        self.axes = [0.0] * len(axes_mapping)
        self.buttons = [False] * len(button_mapping)
        self.dpad = (0, 0)

        self.axis_changed = [False] * len(axes_mapping)
        self.prev_buttons = [False] * len(button_mapping)
        self.prev_dpad = (0, 0)

        super(BuggyJoystick, self).__init__()

    @staticmethod
    def create_mapping(list_mapping):
        """
        Turn a list of strings into a dictionary that maps the strings
        to an index in the list
        """
        dict_mapping = {}
        for index, name in enumerate(list_mapping):
            if bool(name) != False:
                dict_mapping[name] = index
        return dict_mapping

    def update(self):
        # Go through every event pygame sees
        for event in pygame.event.get():
            # if event.type != pygame.NOEVENT:
            #     print(event)
            if event.type == pygame.QUIT:
                return False

            # if an axis event occurred
            if event.type == pygame.JOYAXISMOTION:
                # set value to the axis if it is outside the deadzone
                # else set it to zero
                if event.axis < len(self.axes):
                    value = event.value if abs(event.value) > \
                                           self.dead_zones[
                                               event.axis] else 0.0

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
                self.dpad = event.value

            # if a button was pressed, set it to True and call the button
            # down callback
            elif event.type == pygame.JOYBUTTONDOWN:
                if event.button < len(self.buttons):
                    self.buttons[event.button] = True
                else:
                    raise ValueError(
                        "Unregistered button! '%s'. Please add "
                        "it to your joystick class." % event.button)

            # if a button was released, set it to False and call the button
            # up callback
            elif event.type == pygame.JOYBUTTONUP:
                if event.button < len(self.buttons):
                    self.buttons[event.button] = False
                else:
                    raise ValueError(
                        "Unregistered button! '%s'. Please add "
                        "it to your joystick class." % event.button)

    def get_button(self, name):
        """Get the value of a button using the name as reference"""
        return self.buttons[self.name_to_button[name]]

    def button_updated(self, name):
        prev_button = self.prev_buttons[self.name_to_button[name]]
        button = self.buttons[self.name_to_button[name]]
        if prev_button != button:
            self.prev_buttons[self.name_to_button[name]] = button
            return True
        else:
            return False

    def axis_updated(self, name):
        if self.axis_changed[self.name_to_axis[name]]:
            self.axis_changed[self.name_to_axis[name]] = False
            return True
        else:
            return False

    def get_axis(self, name):
        """Get the value of an axis using the name as reference"""
        return self.axes[self.name_to_axis[name]]

    def fill_line(self, line):
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
