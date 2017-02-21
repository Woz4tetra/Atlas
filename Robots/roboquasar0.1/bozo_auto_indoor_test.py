# This file is meant to collect data from manual control. Terminal interface only
from atlasbuggy.interface.live import LiveRobot

from joysticks.wiiu_joystick import WiiUJoystick
from algorithms.controls.bozo_controller import BozoController
from roboquasar import RoboQuasar

roboquasar = RoboQuasar(True, "buggy course map", "buggy course map inside border", "buggy course map outside border")


class DataCollector(LiveRobot):
    def __init__(self):
        self.controller = BozoController([[0, 0], [0, -10]])
        self.controller.initialize(0, 0)

        self.manual_mode = True

        print("X: Switch between manual and autonomous\n"
              "R: Deadman switch. Hold this until something goes wrong\n"
              "L: Record checkpoint\n"
              "left stick: steering while in manual mode\n"
              "A: calibrate steering")

        super(DataCollector, self).__init__(
            *roboquasar.get_sensors(),
            joystick=WiiUJoystick(),
            log_data=False, debug_prints=True
        )

    def start(self):
        roboquasar.brakes.unbrake()

    def received(self, timestamp, whoiam, packet, packet_type):
        # if self.did_receive(roboquasar.gps):
        #     print("%2.5f" % timestamp)
        #     print(roboquasar.gps)
        #     print(roboquasar.imu)

        if self.did_receive(roboquasar.imu):
            self.update_steering()

        elif self.did_receive(roboquasar.steering):
            print("%2.5f" % timestamp)
            print(roboquasar.steering)
        elif self.did_receive(roboquasar.brakes):
            print("%2.5f" % timestamp)
            print(roboquasar.brakes)

    def update_steering(self):
        if not self.manual_mode:
            if self.controller.current_index is None:
                self.controller.initialize(0, 0)
            else:
                steering_angle = self.controller.update(0, 0, roboquasar.offset_angle())
                print("Steering angle:", steering_angle)
                if not self.manual_mode:
                    roboquasar.steering.set_position(steering_angle)

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
