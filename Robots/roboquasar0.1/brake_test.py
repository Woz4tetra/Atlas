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

        self.input_pattern = re.compile("^[0-9]{3}")

        super(BrakeTest, self).__init__(self.gps, self.imu, self.steering, self.brakes)

    def loop(self):
        input_value = input("> ")
        if input_value == "-":
            self.brakes.set_brake(input_value)
        else:
            match = self.input_pattern.match(input_value)
            brake_value = int(match.group(0))
            if abs(brake_value - self.brakes.position) > 10:
                response = input("input value difference is more than 10, are you sure you want to proceed? (y/n)")
                if response == "y":
                    self.brakes.set_brake(brake_value)
                else:
                    print("skipping")
