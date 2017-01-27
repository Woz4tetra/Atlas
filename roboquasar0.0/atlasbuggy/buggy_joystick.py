"""
An interface with any joystick pygame recognizes. This file also serves as a
utility for connecting new joysticks. Simply run this file and follow the
instructions. This class doesn't support simultaneous joystick connections.
Please connect one joystick at a time.
"""

import threading
import time

import pygame
import os

class BuggyJoystick(threading.Thread):
    exit_flag = False  # when True, the thread is stopped

    # TODO: add multiple joystick support
    def __init__(self, axes_mapping, axes_dead_zones, button_mapping,
                 button_down_fn=None, button_up_fn=None,
                 axis_active_fn=None, axis_inactive_fn=None,
                 dpad_active_fn=None, dpad_inactive_fn=None):
        # initialize pygame
        os.environ["SDL_VIDEODRIVER"] = "dummy"  # prevent pygame from launching a window
        pygame.init()
        pygame.joystick.init()

        # search for all available joysticks and initialize them
        joysticks = [pygame.joystick.Joystick(x) for x in
                     range(pygame.joystick.get_count())]
        assert len(joysticks) > 0
        for joy in joysticks:
            joy.init()
            print(joy.get_name(), joy.get_id(), joy.get_init(),
                  joy.get_numaxes())

        # Initialize axes and button mapping variables
        assert type(axes_mapping) == list or type(axes_mapping) == tuple
        assert type(axes_dead_zones) == list or type(axes_dead_zones) == tuple
        assert type(button_mapping) == list or type(button_mapping) == tuple

        assert len(axes_dead_zones) == len(axes_mapping)

        self.axis_to_name = axes_mapping
        self.button_to_name = button_mapping

        # create dictionaries that maps names to button/axis IDs
        self.name_to_axis = self.create_mapping(axes_mapping)
        self.name_to_button = self.create_mapping(button_mapping)

        self.dead_zones = axes_dead_zones

        # current values
        self.axes = [0.0] * len(axes_mapping)
        self.buttons = [False] * len(button_mapping)
        self.prev_buttons = [False] * len(button_mapping)
        self.dpad = (0, 0)

        # callbacks for button presses/axis movement
        self.button_down_fn = button_down_fn
        self.button_up_fn = button_up_fn
        self.axis_active_fn = axis_active_fn
        self.axis_inactive_fn = axis_inactive_fn
        self.dpad_active_fn = dpad_active_fn
        self.dpad_inactive_fn = dpad_inactive_fn

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

    def run(self):
        """Joystick main thread. Exit when exit_flag is True"""
        self.repeat_t0 = time.time()
        self.repeat_t1 = time.time()

        while not BuggyJoystick.exit_flag:
            self.update()
            time.sleep(0.001)

    @staticmethod
    def stop():
        BuggyJoystick.exit_flag = True

    def update(self):
        """
        Update the joystick values and activate the callbacks
        """

        # Go through every event pygame sees
        for event in pygame.event.get():
            # if event.type != pygame.NOEVENT:
            #     print(event)
            if event.type == pygame.QUIT:
                self.stop()

            # if an axis event occurred
            if event.type == pygame.JOYAXISMOTION:
                # set value to the axis if it is outside the deadzone
                # else set it to zero
                if event.axis < len(self.axes):
                    value = event.value if abs(event.value) > self.dead_zones[
                        event.axis] else 0.0

                    # if the value changed
                    if self.axes[event.axis] != value:
                        # if the axis went into the deadzone
                        if value == 0.0:
                            # call the axis inactive callback
                            if self.axis_inactive_fn is not None:
                                self.axis_inactive_fn(
                                    self.axis_to_name[event.axis])
                        # if the axis is outside the deadzone
                        else:
                            # call the axis active callback
                            if self.axis_active_fn is not None:
                                self.axis_active_fn(
                                    self.axis_to_name[event.axis], value)
                        # update the current value
                        self.axes[event.axis] = value
                else:
                    raise ValueError("Unregistered axis! '%s'. Please add "
                                     "it to your joystick class." % event.axis)

            # if the dpad updated, set the current value and call the
            # dpad callback
            elif event.type == pygame.JOYHATMOTION:
                self.dpad = event.value
                if self.dpad != (0, 0):
                    if self.dpad_active_fn is not None:
                        self.dpad_active_fn(self.dpad)
                else:
                    if self.dpad_inactive_fn is not None:
                        self.dpad_inactive_fn()

            # if a button was pressed, set it to True and call the button
            # down callback
            elif event.type == pygame.JOYBUTTONDOWN:
                if event.button < len(self.buttons):
                    self.buttons[event.button] = True
                    if self.button_down_fn is not None:
                        self.button_down_fn(self.button_to_name[event.button])
                else:
                    raise ValueError(
                        "Unregistered button! '%s'. Please add "
                        "it to your joystick class." % event.button)

            # if a button was released, set it to False and call the button
            # up callback
            elif event.type == pygame.JOYBUTTONUP:
                if event.button < len(self.buttons):
                    self.buttons[event.button] = False
                    if self.button_up_fn is not None:
                        self.button_up_fn(self.button_to_name[event.button])
                else:
                    raise ValueError(
                        "Unregistered button! '%s'. Please add "
                        "it to your joystick class." % event.button)

    def get_button(self, name):
        """Get the value of a button using the name as reference"""
        return self.buttons[self.name_to_button[name]]

    def get_axis(self, name):
        """Get the value of an axis using the name as reference"""
        return self.axes[self.name_to_axis[name]]

    def __str__(self):
        """Format the current values of the joystick nicely"""
        string = "axes:"
        counter = 0
        for name, index in self.name_to_axis.items():
            string += name + ": " + str(self.axes[index]) + "\t"
            if counter % 3 == 0 and index != 0:
                string += "\n"
            counter += 1
        string += "\n"

        string += "buttons:\n"
        counter = 0
        for name, index in self.name_to_button.items():
            string += name + ": " + str(self.buttons[index]) + "\t"
            if counter % 5 == 0 and index != 0:
                string += "\n"
            counter += 1
        string += "\n"

        string += "dpad:" + str(self.dpad) + "\n"

        return string


# Joystick creation utility
if __name__ == '__main__':
    import project


    def init():
        """Initialize the utility"""

        # initialize pygame
        pygame.init()
        pygame.display.init()
        pygame.joystick.init()
        pygame.display.set_mode((200, 200))

        # Since we don't have the buggy joystick class, initialize the
        # joysticks manually
        joysticks = [pygame.joystick.Joystick(x) for x in
                     range(pygame.joystick.get_count())]
        assert len(joysticks) > 0
        for joy in joysticks:
            joy.init()
            print(joy.get_name(), joy.get_id(), joy.get_init(),
                  joy.get_numaxes())


    def get_main_event(joy_class_content):
        """Get event the utility should listen to"""
        main_event = None
        highest_value = None
        enter_pressed = False
        esc_pressed = False
        prev_main_event = None

        recorded_axes = []

        # print the axes that have been recorded
        print("\nRecorded axes: ")
        for index, axis in enumerate(joy_class_content["axes"]):
            if axis != "":
                print(axis + ": " + str(index))
                recorded_axes.append(index)

        # print the buttons that have been recorded
        recorded_buttons = []
        print("\nRecorded buttons: ")
        for index, button in enumerate(joy_class_content["buttons"]):
            if button != "":
                print(button + ": " + str(index))
                recorded_buttons.append(index)
        print()

        # wait for the enter or esc key to be pressed
        while enter_pressed == False and esc_pressed == False:
            event = pygame.event.poll()
            if event.type == pygame.QUIT:
                quit()

            # listen for keyboard events. If shift is pressed, it resets
            # the current recording session (it doesn't erase recorded
            # attributes)
            elif event.type == pygame.KEYDOWN:  # or event.type == pygame.KEYUP:
                print(event)
                if event.key == 13:  # enter
                    enter_pressed = True
                if event.key == 303 or event.key == 304:  # shift
                    return None, False, False
                if event.key == 27:  # esc
                    esc_pressed = True

            # Select an event to return. Don't record events that have
            # already been assigned
            elif (event.type == pygame.JOYBUTTONUP or
                          event.type == pygame.JOYBUTTONDOWN) and \
                            event.button not in recorded_buttons:
                main_event = event
            elif event.type == pygame.JOYHATMOTION:
                main_event = event
            elif event.type == pygame.JOYAXISMOTION and \
                            event.axis not in recorded_axes:
                # Only select the highest axis value
                if highest_value is None or abs(event.value) > highest_value:
                    main_event = event
                    highest_value = abs(event.value)

            # don't spam the terminal
            if prev_main_event != main_event:
                prev_main_event = main_event
                print(main_event)

            time.sleep(0.005)

        return main_event, enter_pressed, esc_pressed


    def update_content_button(content, event):
        """If a button was recorded, prompt the user to give it a name"""
        button_name = input("What's this button's name?\n> ")
        while button_name in content["buttons"]:
            button_name = input("Button name already in use...\n> ")
        if button_name != "":  # if the user entered nothing, skip this button
            print(button_name + ": " + str(event.button))

            while len(content["buttons"]) < event.button + 1:
                content["buttons"].append("")

            content["buttons"][event.button] = button_name


    def update_content_axis(content, event):
        """If an axis was recorded, prompt the user to give it a name"""
        axis_name = input("What's this axis' name?\n> ")
        while axis_name in content["axes"]:
            axis_name = input("Axis name already in use...\n> ")
        if axis_name != "":  # if the user entered nothing, skip this axis
            print(axis_name + ": " + str(event.axis))

            while len(content["axes"]) < event.axis + 1:
                content["axes"].append("")
                content["dead zones"].append("")

            content["axes"][event.axis] = axis_name

            dead_zone = None
            while type(dead_zone) != float:
                if dead_zone is None:
                    value = input("What's the axis dead zone value?\n> ")
                else:
                    value = input("Give a decimal number\n> ")

                try:
                    dead_zone = float(value)
                except ValueError:
                    pass

            content["dead zones"][event.axis] = dead_zone


    def main():
        init()

        print("\nAcquiring inputs... press enter when ready to record, "
              "esc to create class and finish, shift to cancel")
        print("Events are prioritized. The last event to be printed when enter "
              "is pressed is the one that will be recorded\n")

        joy_class_content = {
            "axes": [],
            "dead zones": [],
            "buttons": []
        }

        while True:  # records joystick values until the esc key is pressed
            main_event, enter_pressed, esc_pressed = get_main_event(
                joy_class_content)

            if main_event is not None:
                print(main_event)
                if enter_pressed:
                    if main_event.type == pygame.JOYBUTTONUP or \
                                    main_event.type == pygame.JOYBUTTONDOWN:
                        update_content_button(joy_class_content, main_event)

                    elif main_event.type == pygame.JOYAXISMOTION:
                        update_content_axis(joy_class_content, main_event)

            if esc_pressed:  # write the recorded values to a joystick class
                module_name = ""
                while module_name == "":
                    module_name = input(
                        "Enter new joystick file name (without .py extension): ")

                class_name = ""
                while class_name == "":
                    class_name = input(
                        "Enter new joystick class name: ")

                project_name = ""
                while project_name == "":
                    project_name = input(
                        "Enter Atlas project name (for example: rccar): ")

                # remove whitespace
                class_name = class_name.replace(" ", "")
                module_name = module_name.replace(" ", "")

                content_string = """
from atlasbuggy.buggy_joystick import BuggyJoystick


class %s(BuggyJoystick):
    def __init__(self, button_down_fn=None, button_up_fn=None,
                 axis_active_fn=None, axis_inactive_fn=None, joy_hat_fn=None,
                 fn_params=None):
        super(%s, self).__init__(
            %s,
            %s,
            %s,
            button_down_fn, button_up_fn, axis_active_fn,
            axis_inactive_fn, joy_hat_fn, fn_params
        )


if __name__ == '__main__':
    import time

    joystick = %s()
    joystick.start()

    try:
        while True:
            print(joystick)
            time.sleep(0.15)
    except KeyboardInterrupt:
        joystick.stop()
""" % (class_name, class_name, joy_class_content["axes"],
       joy_class_content["dead zones"], joy_class_content["buttons"],
       class_name)
                project.set_project_dir(project_name)

                with open(project.get_dir(":joysticks") + module_name + ".py",
                          "w") as joy_module:
                    joy_module.write(content_string)

                print("Done! Exiting.")

                break


    main()
