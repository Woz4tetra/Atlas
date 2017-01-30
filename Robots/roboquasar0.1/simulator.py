from atlasbuggy.robot.simulator import RobotInterfaceSimulator

from sensors.gps import GPS
from sensors.imu import IMU
from actuators.steering import Steering


class Simulator(RobotInterfaceSimulator):
    def __init__(self):
        self.gps = GPS()
        self.imu = IMU()
        self.steering = Steering()

        super(Simulator, self).__init__(
            -1, -1, self.gps, self.imu, self.steering,
            # start_index=1000,
            # end_index=2000,
        )

    def object_packet(self, timestamp):
        if self.did_receive(self.imu):
            print(timestamp, self.imu.eul_x, self.imu.accel_x, self.imu.gyro_x, self.imu.mag_x)

Simulator().run()
