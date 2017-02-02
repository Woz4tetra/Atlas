
from atlasbuggy.robot.interface import RobotInterface
from sensors.imu import IMU

class IMUtester(RobotInterface):
    def __init__(self):
        self.imu = IMU()
        super(IMUtester, self).__init__(self.imu, log_data=False)

    def packet_received(self, timestamp, whoiam, packet):
        pass

    def loop(self):
        pass

IMUtester().run()