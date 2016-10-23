import time

from autobuggy import project
from autobuggy.robot import Robot
from autobuggy.microcontroller.logger import get_map
from joysticks.wiiu_joystick import WiiUJoystick
from autobuggy.filters.groves_kf import KF


class RoboQuasarBot(Robot):
    def __init__(self, log_data=True):
        # set the project name (so that maps and logs and be found)
        project.set_project_dir("roboquasar")

        self.manual_mode = True

        self.goal_x, self.goal_y = 0, 0
        self.checkpoints = get_map("checkpoints.txt")
        self.checkpoint_num = 0

        joystick = WiiUJoystick(
            button_down_fn=lambda button, params: self.button_dn(
                button, params),
            axis_active_fn=lambda axis, value, params: self.axis_active(
                axis, value, params),
            axis_inactive_fn=lambda axis, params: self.axis_inactive(
                axis, params),
        )

        sensors = dict(
            gps=dict(sensor_id=1, properties=['long', 'lat', 'altitude',
                                              'speed', 'course', 'pdop'],
                     update_fn=lambda: self.gps_updated()),
            imu=dict(sensor_id=2, properties=['ax', 'ay', 'az',
                                              'gx', 'gy', 'gz'],
                     update_fn=lambda: self.imu_updated()),
        )

        commands = dict(
            leds=dict(command_array={
                "red": 0,
                "yellow": 1,
                "green": 2,
            }, range=(0, 2),
                mapping={
                    "off": 0,
                    "on": 1,
                    "toggle": 2
                }),
            blue_led=dict(command_id=3, range=(0, 255)),
        )

        self.filter = KF()
        # filter.get_position()

        super(RoboQuasarBot, self).__init__(
            sensors, commands, '/dev/ttyUSB0', joystick=joystick,
            close_fn=self.close_fn, log_data=log_data, log_dir=":today"
        )

        self.imu = self.sensors['imu']
        self.gps = self.sensors['gps']

        self.blue_led = self.commands['blue_led']

        self.prev_time = time.time()

    def button_dn(self, button, params):
        pass

    def axis_inactive(self, axis, params):
        pass

    def axis_active(self, axis, value, params):
        pass

    def imu_updated(self):
        self.filter.update_imu(
            [[self.imu.get('ax'), self.imu.get('ay'), self.imu.get('az')],
             [self.imu.get('gx'), self.imu.get('gy'), self.imu.get('gz')]]
        )  # [[force_x, force_y, force_z], [angular roll, angular pitch, angular yaw]]
        print(self.imu.get(all=True))

    def gps_updated(self):
        x_ecef = self.gps.get('long')
        y_ecef = self.gps.get('lat')
        z_ecef = self.gps.get('altitude')

        vx_ecef = self.gps.get('speed')
        vy_ecef = self.gps.get('speed')
        vz_ecef = self.gps.get('speed')

        self.filter.update_gps(
            [x_ecef, y_ecef, z_ecef],
            [vx_ecef, vy_ecef, vz_ecef],
            time.time() - self.prev_time,
            self.gps.get('pdop')
        )  # [x, y, z], [vx, vy, vz], dt, pdop

        self.prev_time = time.time()
        print(self.gps.get(all=True))

    def close_fn(self):
        self.blue_led.set(0)
