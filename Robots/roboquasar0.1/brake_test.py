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
        brake_value = input("> ")
        match = self.input_pattern.match(brake_value)
        self.brakes.set_position(match.group(0))

