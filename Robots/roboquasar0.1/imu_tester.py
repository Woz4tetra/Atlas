
from atlasbuggy.robot.interface import RobotInterface
from atlasbuggy.robot.simulator import RobotInterfaceSimulator
from sensors.imu import IMU

class IMUrunner(RobotInterface):
    def __init__(self):
        self.imu = IMU()
        super(IMUrunner, self).__init__(self.imu, log_data=False, debug_prints=True)

    def packet_received(self, timestamp, whoiam, packet):
        pass

    def loop(self):
        pass

class IMUsimatulor(RobotInterfaceSimulator):
    def __init__(self):
        self.imu = IMU()
        super(IMUsimatulor, self).__init__(self.imu, -1, -1)

    # def object_received(self, timestamp):
        # self.imu

simulate = False

if simulate:
    IMUsimatulor().run()
else:
    IMUrunner().run()
