# This file is meant to collect data from manual control. Terminal interface only
import argparse

from atlasbuggy.interface.live import LiveRobot

from sensors.gps import GPS
from sensors.imu import IMU

from actuators.brakes import Brakes
from actuators.steering import Steering
from actuators.underglow import Underglow

from joysticks.wiiu_joystick import WiiUJoystick

parser = argparse.ArgumentParser()
parser.add_argument("-l", "--nolog", help="disable logging", action="store_false")
args = parser.parse_args()


class DataCollector(LiveRobot):
    def __init__(self):
        self.gps = GPS()
        self.imu = IMU()

        self.steering = Steering()
        self.brakes = Brakes()
        self.underglow = Underglow()

        self.checkpoint_num = 0

        super(DataCollector, self).__init__(
            self.gps, self.imu, self.steering, self.brakes, self.underglow,
            # joystick=WiiUJoystick(),
            log_data=args.nolog, log_dir=("rolls", None), debug_prints=True
        )

    def received(self, timestamp, whoiam, packet, packet_type):
        if self.did_receive(self.gps):
            print("%2.5f" % timestamp)
            print(self.gps)
            print(self.imu)
        elif self.did_receive(self.steering):
            print("%2.5f" % timestamp)
            print(self.steering)
        elif self.did_receive(self.brakes):
            print("%2.5f" % timestamp)
            print(self.brakes)

    def loop(self):
        if self.joystick is not None:
            if self.steering.calibrated:
                if self.joystick.axis_updated("left x"):
                    self.steering.set_speed(self.joystick.get_axis("left x"))
                elif self.joystick.button_updated("A") and self.joystick.get_button("A"):
                    self.steering.set_position(0)

            if self.joystick.button_updated("B") and self.joystick.get_button("B"):
                self.brakes.brake()
            elif self.joystick.button_updated("X") and self.joystick.get_button("X"):
                self.brakes.unbrake()
            elif self.joystick.button_updated("L") and self.joystick.get_button("L"):
                print("Current checkpoint:", self.checkpoint_num)
                print(self.gps)
                self.record("checkpoint", str(self.checkpoint_num))
                self.checkpoint_num += 1


DataCollector().run()
