import time

from atlasbuggy import project
from atlasbuggy.robot import Robot
from atlasbuggy.microcontroller.logger import get_map
from joysticks.wiiu_joystick import WiiUJoystick
from controller import Controller

class RoboQuasarBot(Robot):
    def __init__(self, checkpoints_name, map_name, log_data=True):
        # set the project name (so that maps and logs and be found)
        project.set_project_dir("roboquasar")

        self.manual_mode = True

        self.goal_x, self.goal_y = 0, 0
        self.checkpoints = get_map(checkpoints_name)
        self.checkpoint_num = 0

        self.map = get_map(map_name)

        self.controller = Controller(self.map)

        joystick = WiiUJoystick(
            button_down_fn=lambda button: self.button_dn(button),
            # axis_active_fn=lambda axis, value: self.axis_active(axis, value),
             axis_inactive_fn=lambda axis: self.axis_inactive(axis),
            dpad_active_fn=lambda direction: self.dpad(direction),
        )

        sensors = dict(
            gps=dict(sensor_id=1,
                     properties=['lat', 'long', 'altitude', 'geoid_height',
                                 'pdop', 'hdop', 'vdop', 'fix'],
                     update_fn=lambda: self.gps_updated()),
            imu=dict(sensor_id=2, properties=[
                'yaw', 'ax', 'ay', 'az', 'gx', 'gy', 'gz', 'mx', 'my', 'mz'],
                     update_fn=lambda: self.imu_updated()),
        )

        commands = dict(
            leds=dict(command_array={
                "red": 0,
                "yellow": 1,
                "green": 2,
            },
                range=(0, 2),
                mapping={
                    "off": 0,
                    "on": 1,
                    "toggle": 2
                }),
            blue_led=dict(command_id=3, range=(0, 255)),
            stepper=dict(command_id=4, range=(-0x8000, 0x7fff)),
        )

        super(RoboQuasarBot, self).__init__(
            sensors, commands, '/dev/ttyUSB0', joystick=joystick,
            close_fn=self.close_fn, log_data=log_data, log_dir=":today"
        )

        self.imu = self.sensors['imu']
        self.gps = self.sensors['gps']
        
        self.leds = self.commands['leds']
        self.blue_led = self.commands['blue_led']

        self.stepper = self.commands['stepper']

        self.prev_time = time.time()

    def button_dn(self, button):
        pass

    def axis_inactive(self, axis):
        if axis == "left x":
            print("joystick inactive")
            self.blue_led.set(0)

    def axis_active(self, axis, value):
        pass

    def dpad(self, direction):
        pass

    def imu_updated(self):
        pass

    def gps_updated(self):
        pass

    def close_fn(self):
        self.blue_led.set(0)
