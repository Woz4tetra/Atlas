# Compare checkpoint data to checkpoint maps
import time

from atlasbuggy.interface.simulated import SimulatedRobot

from sensors.gps import GPS
from sensors.imu import IMU
from sensors.lidarturret import LidarTurret

from actuators.brakes import Brakes
from actuators.steering import Steering
from actuators.underglow import Underglow

from roboquasar import file_sets


class FilePrinter(SimulatedRobot):
    def __init__(self):
        self.gps = GPS()
        self.imu = IMU()
        self.lidar = LidarTurret()

        self.steering = Steering()
        self.brakes = Brakes()
        self.underglow = Underglow()

        file_name, directory = file_sets["data day 5"][-1]
        super(FilePrinter, self).__init__(
            file_name, directory,
            self.gps, self.imu, self.lidar, self.steering, self.brakes, self.underglow
        )

    def received(self, timestamp, whoiam, packet, packet_type):
        if self.did_receive(self.gps):
            print("%2.5f" % timestamp)
            print(self.gps)
        elif self.did_receive(self.imu):
            print("%2.5f" % timestamp)
            print(self.imu)
        elif self.did_receive(self.steering):
            print("%2.5f" % timestamp)
            print(self.steering)
        elif self.did_receive(self.brakes):
            print("%2.5f" % timestamp)
            print(self.steering)
        else:
            print("%2.5f\t%s\t%s" % (timestamp, whoiam, repr(packet)))
        time.sleep(0.001)


FilePrinter().run()
