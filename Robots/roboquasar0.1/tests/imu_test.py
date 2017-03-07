from atlasbuggy.interface.live import RobotRunner
from atlasbuggy.interface.simulated import RobotSimulator
from sensors.imu import IMU


class IMUrunner(RobotRunner):
    def __init__(self):
        self.imu = IMU()
        super(IMUrunner, self).__init__(self.imu, log_data=True, debug_prints=True)

    def updated(self, timestamp, whoiam, packet):
        pass

    def loop(self):
        pass


class IMUsimatulor(RobotSimulator):
    def __init__(self):
        self.imu = IMU()
        super(IMUsimatulor, self).__init__(-1, -1, self.imu)

        # def object_received(self, timestamp):
        # self.imu


simulate = True

if simulate:
    IMUsimatulor().run()
else:
    IMUrunner().run()
