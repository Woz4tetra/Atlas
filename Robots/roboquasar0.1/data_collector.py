# This file is meant to collect data from manual control. Terminal interface only
import argparse

from atlasbuggy.interface.live import LiveRobot

from roboquasar import RoboQuasar
from joysticks.wiiu_joystick import WiiUJoystick

parser = argparse.ArgumentParser()
parser.add_argument("-l", "--nolog", help="disable logging", action="store_false")
args = parser.parse_args()


roboquasar = RoboQuasar(True, "buggy course map")


class DataCollector(LiveRobot):
    def __init__(self):
        super(DataCollector, self).__init__(
            *roboquasar.get_sensors(),
            joystick=WiiUJoystick(),
            log_data=args.nolog, log_dir=("rolls", None), debug_prints=True
        )

    def received(self, timestamp, whoiam, packet, packet_type):
        if self.did_receive(roboquasar.gps):
            print("%2.5f" % timestamp)
            print(roboquasar.gps)
            print(roboquasar.imu)
        elif self.did_receive(roboquasar.steering):
            print("%2.5f" % timestamp)
            print(roboquasar.steering)
        elif self.did_receive(roboquasar.brakes):
            print("%2.5f" % timestamp)
            print(roboquasar.brakes)

    def loop(self):
        if self.joystick is not None:
            if roboquasar.steering.calibrated:
                if self.joystick.axis_updated("left x"):
                    roboquasar.steering.set_speed(self.joystick.get_axis("left x"))
                elif self.joystick.button_updated("A") and self.joystick.get_button("A"):
                    roboquasar.steering.calibrate()

            if self.joystick.button_updated("B") and self.joystick.get_button("B"):
                roboquasar.brakes.brake()
            elif self.joystick.button_updated("X") and self.joystick.get_button("X"):
                roboquasar.brakes.unbrake()
            elif self.joystick.button_updated("L") and self.joystick.get_button("L"):
                print("Current checkpoint:", self.checkpoint_num)
                print(roboquasar.gps)
                self.record("checkpoint", str(self.checkpoint_num))
                self.checkpoint_num += 1

    def close(self, reason):
        if reason != "done":
            roboquasar.brakes.brake()
            print("!!EMERGENCY BRAKE!!")

DataCollector().run()
