from atlasbuggy.robot.simulator import RobotInterfaceSimulator
from atlasbuggy.plotters.staticplotter import StaticPlotter
from atlasbuggy.plotters.robotplot import RobotPlot

from sensors.gps import GPS
from sensors.imu import IMU
from actuators.steering import Steering
from actuators.brakes import Brakes


class Simulator(RobotInterfaceSimulator):
    def __init__(self):
        self.gps = GPS()
        self.imu = IMU()
        self.steering = Steering()
        self.brakes = Brakes()

        self.imuPlot = RobotPlot("IMU Data")
        self.gpsPlot = RobotPlot("GPS Data")

        self.staticPlot = StaticPlotter(2, self.imuPlot, self.gpsPlot)

        super(Simulator, self).__init__(
            -1, -1, self.gps, self.imu, self.steering, self.brakes,
            # start_index=1000,
            # end_index=2000,
        )

    def object_packet(self, timestamp):
        print(self.packet)
        if self.did_receive(self.imu):
            self.imuPlot.append(self.imu.euler.x, self.imu.euler.y, self.imu.euler.z)
            self.gpsPlot.append(self.gps.latitude, self.gps.longitude, self.gps.altitude)

    def close(self):
        self.staticPlot.plot()
        self.staticPlot.show()


Simulator().run()
