from gps import GPS
from imu import IMU
from atlasbuggy.robot.interface import RobotInterface


class Runner(RobotInterface):
    def __init__(self):
        self.gps = GPS()
        self.imu = IMU()

        super(Runner, self).__init__(
            self.imu, self.gps,
            debug_prints=True,
            log_data=False
        )

    def packet_received(self, timestamp, whoiam, packet):
        pass

    def loop(self):
        pass


Runner().run()
