# This file is meant to collect data from manual control. Terminal interface only
import math

from atlasbuggy.interface.live import LiveRobot
from atlasbuggy.files.mapfile import MapFile

from sensors.gps import GPS
from sensors.imu import IMU

from actuators.brakes import Brakes
from actuators.steering import Steering
from actuators.underglow import Underglow

from joysticks.wiiu_joystick import WiiUJoystick

from algorithms.controls.bozo_controller import BozoController


class DataCollector(LiveRobot):
    def __init__(self):
        self.gps = GPS()
        self.imu = IMU()

        self.steering = Steering()
        self.brakes = Brakes()
        self.underglow = Underglow()

        self.controller = BozoController([[0, 0], [10, 0]])

        self.checkpoint_num = 0
        self.manual_mode = True
        self.safety_enabled = False

        super(DataCollector, self).__init__(
            self.gps, self.imu, self.steering, self.brakes, self.underglow,
            joystick=WiiUJoystick(),
            log_data=False, debug_prints=True
        )

        self.start_angle = None

    def received(self, timestamp, whoiam, packet, packet_type):
        if self.did_receive(self.gps):
            print("%2.5f" % timestamp)
            print(self.gps)
            print(self.imu)

        elif self.did_receive(self.imu):
            if self.start_angle is None:
                self.start_angle = self.imu.euler.z
            self.update_steering()
            print("Steering angle:", self.controller.current_angle)

        elif self.did_receive(self.steering):
            print("%2.5f" % timestamp)
            print(self.steering)
        elif self.did_receive(self.brakes):
            print("%2.5f" % timestamp)
            print(self.brakes)

    def update_steering(self):
        if self.controller.current_index is None:
            self.controller.initialize(0, 0)
        else:
            steering_angle = self.controller.update(self.gps.latitude_deg, self.gps.longitude_deg,
                                                    -(self.imu.euler.z - self.start_angle))
            if not self.manual_mode:
                self.steering.set_position(steering_angle)

    def loop(self):
        if self.joystick is not None:
            if self.steering.calibrated:
                if self.manual_mode:
                    if self.joystick.axis_updated("left x"):
                        self.steering.set_speed(self.joystick.get_axis("left x"))
                    elif self.joystick.button_updated("A") and self.joystick.get_button("A"):
                        self.steering.set_position(0)

            if self.joystick.button_updated("X") and self.joystick.get_button("X"):
                self.manual_mode = not self.manual_mode
                print("Manual control enabled" if self.manual_mode else "Autonomous mode enabled")
            elif self.joystick.button_updated("L") and self.joystick.get_button("L"):
                print("Current checkpoint:", self.checkpoint_num)
                print(self.gps)
                self.record("checkpoint", str(self.checkpoint_num))
                self.checkpoint_num += 1

            if self.safety_enabled:
                if not self.joystick.get_button("R"):
                    self.brakes.brake()
                    print("!!SWITCH RELEASED, BRAKING!!")

                if self.joystick.button_updated("R") and self.joystick.get_button("R"):
                    self.brakes.unbrake()

    def close(self, reason):
        if reason != "done" and self.safety_enabled:
            self.brakes.brake()
            print("!!EMERGENCY BRAKE!!")


DataCollector().run()
