# This file is meant to collect data from manual control. Terminal interface only
from atlasbuggy.interface.live import LiveRobot

from joysticks.wiiu_joystick import WiiUJoystick
from algorithms.controls.bozo_controller import BozoController
from roboquasar import RoboQuasar

roboquasar = RoboQuasar(True, "Autonomous test map 1", "Autonomous test map 1 inside border",
                        "Autonomous test map 1 outside border", "cut")


class DataCollector(LiveRobot):
    def __init__(self):
        self.controller = BozoController(roboquasar.checkpoints.map)

        self.manual_mode = True

        print("X: Switch between manual and autonomous\n"
              "R: Deadman switch. Hold this until something goes wrong\n"
              "L: Record checkpoint\n"
              "left stick: steering while in manual mode\n"
              "A: calibrate steering"
              )
        super(DataCollector, self).__init__(
            *roboquasar.get_sensors(),
            joystick=WiiUJoystick(),
            log_data=True, log_dir=("rolls", None), debug_prints=True
        )
        self.record("initial compass", roboquasar.compass_str)

    def start(self):
        roboquasar.brakes.unbrake()

    def received(self, timestamp, whoiam, packet, packet_type):
        if self.did_receive(roboquasar.gps):
            print("%2.5f" % timestamp)
            print(roboquasar.gps)
            print(roboquasar.imu)

            self.update_steering()

        elif self.did_receive(roboquasar.imu):
            self.update_steering()

        elif self.did_receive(roboquasar.steering):
            print("%2.5f" % timestamp)
            print(roboquasar.steering)
        elif self.did_receive(roboquasar.brakes):
            print("%2.5f" % timestamp)
            print(roboquasar.brakes)

    def update_steering(self):
        if roboquasar.gps.is_position_valid():
            if not self.controller.is_initialized():
                self.controller.initialize(roboquasar.gps.latitude_deg, roboquasar.gps.longitude_deg)
            else:
                steering_angle = self.controller.update(roboquasar.gps.latitude_deg, roboquasar.gps.longitude_deg,
                                                        roboquasar.offset_angle())
                if not self.manual_mode:
                    roboquasar.steering.set_position(steering_angle)

                self.record("steering angle",
                            "%s\t%s\t%s" % (steering_angle, self.manual_mode, self.controller.current_index))
        else:
            print("Skipping GPS value!!")

    def loop(self):
        if self.joystick is not None:
            if roboquasar.steering.calibrated and self.manual_mode:
                if self.joystick.axis_updated("left x"):
                    roboquasar.steering.set_speed(self.joystick.get_axis("left x"))
                elif self.joystick.button_updated("A") and self.joystick.get_button("A"):
                    roboquasar.steering.calibrate()

            if self.joystick.button_updated("X") and self.joystick.get_button("X"):
                self.manual_mode = not self.manual_mode
                print("Manual control enabled" if self.manual_mode else "Autonomous mode enabled")
            elif self.joystick.button_updated("L") and self.joystick.get_button("L"):
                print("Current checkpoint:", roboquasar.checkpoint_num)
                print(roboquasar.gps)
                self.record("checkpoint", str(roboquasar.checkpoint_num))
                roboquasar.checkpoint_num += 1

            if self.joystick.button_updated("R"):
                if self.joystick.get_button("R"):
                    roboquasar.brakes.unbrake()
                else:
                    roboquasar.brakes.brake()
                    print("!!SWITCH RELEASED, BRAKING!!")

    def close(self, reason):
        if reason != "done":
            roboquasar.brakes.brake()
            print("!!EMERGENCY BRAKE!!")


DataCollector().run()
