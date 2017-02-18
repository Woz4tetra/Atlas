from atlasbuggy.robot.interface import RobotInterface
from atlasbuggy.robot.simulator import RobotInterfaceSimulator

from atlasbuggy.plotters.robotplot import RobotPlot, RobotPlotCollection
from atlasbuggy.plotters.staticplotter import StaticPlotter

from joysticks.wiiu_joystick import WiiUJoystick

from sensors.gps import GPS
from sensors.imu import IMU

from actuators.steering import Steering
from actuators.brakes import Brakes
from actuators.underglow import Underglow


class CheckpointFinder(RobotInterface):
    def __init__(self):
        self.gps = GPS()
        self.imu = IMU()
        self.steering = Steering()
        self.brakes = Brakes()
        self.underglow = Underglow()

        super(CheckpointFinder, self).__init__(
            self.gps,
            self.imu,
            self.steering,
            self.brakes,
            self.underglow,
            joystick=WiiUJoystick(),
            log_data=False,
        )

        self.checkpoint_num = 0

    def loop(self):
        if self.joystick is not None:
            if self.steering.calibrated:
                if self.joystick.axis_updated("left x"):
                    self.steering.set_speed(self.joystick.get_axis("left x"))
                elif self.joystick.button_updated("A") and self.joystick.get_button("A"):
                    self.steering.set_position(0)

            if self.joystick.button_updated("B") and self.joystick.get_button("B"):
                self.brakes.brake()
            elif self.joystick.button_updated("X") and self.joystick.get_button("X"):
                self.brakes.unbrake()
            elif self.joystick.button_updated("L") and self.joystick.get_button("L"):
                print("Current checkpoint:", self.checkpoint_num)
                print(self.gps)
                self.record("checkpoint", str(self.checkpoint_num))
                self.checkpoint_num += 1


class CheckpointSimulator(RobotInterfaceSimulator):
    def __init__(self):
        self.gps = GPS()

        self.checkpoint_plot = RobotPlot("checkpoints", marker='.', markersize=5, linestyle='')
        self.gps_plot = RobotPlot("gps")
        self.check_gps = RobotPlotCollection("check gps", self.checkpoint_plot, self.gps_plot)

        self.static_plot = StaticPlotter(1, self.check_gps)

        super(CheckpointSimulator, self).__init__(
            "17;37;30", "2017_Feb_17",
            self.gps
        )

    def object_packet(self, timestamp):
        if self.did_receive(self.gps):
            if self.gps.fix and -80 < self.gps.longitude_deg < -79.8 and 40.4 < self.gps.latitude_deg < 41 and (
                                280 < self.gps.altitude < 310 or self.gps.altitude == 0.0):
                self.gps_plot.append(self.gps.latitude_deg, self.gps.longitude_deg)

    def user_packet(self, timestamp, packet):
        if self.did_receive("checkpoint"):
            if self.gps.fix and -80 < self.gps.longitude_deg < -79.8 and 40.4 < self.gps.latitude_deg < 41 and (
                                280 < self.gps.altitude < 310 or self.gps.altitude == 0.0):
                self.checkpoint_plot.append(self.gps.latitude_deg, self.gps.longitude_deg)
                print(self.gps.latitude_deg, self.gps.longitude_deg)

    def close(self):
        self.static_plot.plot()
        self.static_plot.show()




CheckpointFinder().run()
# CheckpointSimulator().run()
