import os
import sys

sys.path.insert(0, '../')

from microcontroller.comm import *
from microcontroller.data import *
from vision.pipeline import Pipeline
from vision.camera import Camera
from manual.wiiu_joystick import WiiUJoystick
from navigation.buggypi_filter import BuggyPiFilter
from robots.robot import Robot


class RealRobot(Robot):
    def __init__(self, properties, sensors, commands):
        super(RealRobot, self).__init__(properties)

        # ----- user set properties -----

        #       ----- general properties -----
        self.close_fn = self.get_property(properties, 'close_fn',
                                          lambda robot: robot)
        self.turn_display_off = self.get_property(
            properties, 'turn_display_off', False)
        self.has_autonomous = self.get_property(
            properties, 'has_autonomous', True)
        self.manual_mode = self.get_property(
            properties, 'manual_mode', True)

        #       ----- camera -----
        self.enable_camera = self.get_property(
            properties, 'enable_camera', True)
        self.cam_width = self.get_property(
            properties, 'cam_width', ValueError)
        self.cam_height = self.get_property(
            properties, 'cam_height', ValueError)
        self.use_cv_pipeline = self.get_property(
            properties, 'use_cv_pipeline', False)
        self.update_camera_fn = self.get_property(
            properties, 'update_camera_fn', False)

        #       ----- logger -----
        self.log_data = self.get_property(properties, 'log_data', True)
        self.log_dir = self.get_property(properties, 'log_dir')
        self.record_state = self.get_property(properties, 'record_state', True)

        #       ----- checkpoints -----
        self.checkpoints_file = self.get_property(properties,
                                                  'checkpoints_file')
        self.checkpoints_dir = self.get_property(properties, 'checkpoints_dir')

        #       ----- joystick-----
        self.use_joystick = self.get_property(properties, 'use_joystick')

        self.button_down_fn = self.get_property(properties,
                                                'button_down_fn')
        self.button_up_fn = self.get_property(properties, 'button_up_fn')
        self.axis_active_fn = self.get_property(properties,
                                                'axis_active_fn')
        self.axis_inactive_fn = self.get_property(properties,
                                                  'axis_inactive_fn')
        self.joy_hat_fn = self.get_property(properties, 'joy_hat_fn')

        #
        # ----- initialize internal properties ----- #
        #

        #       ------ sensors -----
        sensor_pool = SensorPool()

        for name, sensor_properties in sensors.items():
            sensor = Sensor(sensor_properties['sensor_id'],
                            name, sensor_properties['update_fn'],
                            sensor_properties['properties'])
            setattr(self, name, sensor)
            sensor_pool.add_sensor(sensor)

        self.communicator = Communicator(
            sensor_pool, self, address='/dev/ttyAMA0',
            log_data=self.log_data,
            log_dir=self.log_dir)
        if not self.communicator.initialized:
            raise Exception("Communicator not initialized...")

        # add all commands
        for name, command_properties in commands.items():
            if 'command_array' in command_properties:
                command_ids = command_properties['command_array']
                command_range = command_properties['range']
                del command_properties['command_array'], \
                    command_properties['range']
                command = CommandArray(command_ids, name, command_range,
                                       self.communicator,
                                       **command_properties)
            else:
                command_id = command_properties['command_id']
                command_range = command_properties['range']
                del command_properties['command_id'], \
                    command_properties['range']
                command = Command(command_id, name, command_range,
                                  self.communicator,
                                  **command_properties)

            setattr(self, name, command)

        # ----- camera -----
        if self.use_cv_pipeline:
            pipeline = Pipeline(self.cam_width, self.cam_height,
                                self.enable_draw)
        else:
            pipeline = None
        if self.enable_camera:
            self.capture = Camera(
                self.cam_width, self.cam_height, self.update_camera_fn,
                enable_draw=self.enable_draw, pipeline=pipeline,
                fn_params=self, framerate=32)
        self.camera_running = True

        # ----- Turn display off? -----
        if not self.enable_draw:
            print("Display will now turn off")
            time.sleep(2)
            self.display_backlight(False)

        # ----- Joystick -----
        if self.use_joystick:
            self.joystick = WiiUJoystick(
                button_down_fn=self.button_down_fn,
                button_up_fn=self.button_up_fn,
                axis_active_fn=self.axis_active_fn,
                axis_inactive_fn=self.axis_inactive_fn,
                joy_hat_fn=self.joy_hat_fn,
                fn_params=self
            )

        # ----- Start external threads -----
        self.communicator.start()
        if self.use_joystick:
            self.joystick.start()
        if self.enable_camera:
            self.capture.start()

        # ----- filter -----
        if self.use_filter:
            if self.initial_long == 'gps' or self.initial_lat == 'gps':
                if self.initial_long == 'gps':
                    while not self.gps.get('fix'):
                        print(self.gps)
                        time.sleep(0.15)
                    self.initial_long = self.gps.get('long')

                if self.initial_lat == 'gps':
                    while not self.gps.get('fix'):
                        time.sleep(0.15)
                    self.initial_lat = self.gps.get('lat')

                print(self.initial_long, self.initial_lat)
                # reinitialize filter with new data
                self.filter = BuggyPiFilter(
                    self.initial_long, self.initial_lat, self.initial_heading,
                    self.counts_per_rotation, self.wheel_radius,
                    self.front_back_dist, self.max_speed,
                    self.left_angle_limit, self.right_angle_limit,
                    self.left_servo_limit, self.right_servo_limit
                )

        self.communicator.enable_callbacks = True
        self.time_start = time.time()
        self.running = True

    def record(self, name, value, **values):
        self.communicator.record(name, value, **values)

    def get_state(self):
        return self.filter.state

    def display_backlight(self, show):
        if not self.enable_draw:
            if show is True:
                os.system(
                    "sudo echo 0 > /sys/class/backlight/rpi_backlight/bl_power")
            else:
                os.system(
                    "sudo echo 1 > /sys/class/backlight/rpi_backlight/bl_power")

    def close(self):
        self.motors.set(0)
        self.servo.set(0)

        if self.enable_camera:
            self.capture.stop()
        time.sleep(0.005)

        if self.use_joystick:
            self.joystick.stop()
        time.sleep(0.005)

        self.communicator.stop()

        self.display_backlight(True)
        self.running = False

        self.close_fn(self)
