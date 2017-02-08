from atlasbuggy.robot.interface import RobotInterface
from sensors.gps import GPS
from sensors.imu import IMU
from actuators.steering import Steering
from actuators.brakes import Brakes
import re


class BrakeTest(RobotInterface):
    def __init__(self):
        self.gps = GPS(enabled=False)
        self.imu = IMU(enabled=False)
        self.steering = Steering(enabled=False)
        self.brakes = Brakes()

        super(BrakeTest, self).__init__(
            self.gps, self.imu, self.steering, self.brakes,
            log_data=False, debug_prints=True
        )

    def loop(self):
        input_value = input("> ")
        if input_value == "-":
            self.brakes.set_brake(input_value)
        else:
            try:
                brake_value = int(input_value)
                if abs(brake_value - self.brakes.position) >= 20:
                    response = input("input value difference is more than 20, are you sure you want to proceed? (y/n)")
                    if response == "y":
                        self.brakes.set_brake(brake_value)
                    else:
                        print("skipping")
            except ValueError:
                self.brakes.set_brake(input_value)

            else:
                print("invalid input:", input_value)
BrakeTest().run()