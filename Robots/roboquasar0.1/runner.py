
from atlasbuggy.plotters.liveplotter import LivePlotter
from atlasbuggy.plotters.robotplot import RobotPlot
from atlasbuggy.robot.interface import RobotInterface

from joysticks.wiiu_joystick import WiiUJoystick

from sensors.gps import GPS
from sensors.imu import IMU
from actuators.steering import Steering

class Runner(RobotInterface):
    def __init__(self):
        self.gps = GPS(enabled=False)
        self.imu = IMU()
        self.steering = Steering()

        self.imu_plot_eul = RobotPlot("imu eul", flat_plot=False, max_length=30)
        self.imu_plot_gyro = RobotPlot("imu gyro", flat_plot=False, max_length=30)
        self.imu_plot_mag = RobotPlot("imu mag", flat_plot=False, max_length=30)
        self.imu_plot_accel = RobotPlot("imu accel", flat_plot = False, max_length=30)

        self.plotter = LivePlotter(2, self.imu_plot_eul, self.imu_plot_gyro, self.imu_plot_mag)

        super(Runner, self).__init__(
            self.imu,
            self.gps,
            self.steering,
            joystick=WiiUJoystick(),
            debug_prints=True,
            log_data=False
        )

    def packet_received(self, timestamp, whoiam, packet):
        if self.did_receive(self.imu):
            # print(timestamp, self.imu.x, self.imu.y, self.imu.z)

            if self.queue_len() < 25:
                self.imu_plot_eul.append(self.imu.eul_x, self.imu.eul_y, self.imu.eul_z)
                self.imu_plot_mag.append(self.imu.mag_x, self.imu.mag_y, self.imu.mag_z)
                self.imu_plot_gyro.append(self.imu.gyro_x, self.imu.mag_y, self.imu.mag_z)
                self.imu_plot_accel.append(self.imu.accel_x, self.imu.accel_y, self.imu.accel_z)
                if self.plotter.plot() is False:
                    return False
            elif self.did_receive(self.gps):
                print("--")

    def loop(self):
        if self.joystick is not None:
            if self.joystick.axis_updated("left x"):
                self.steering.set_speed(self.joystick.get_axis("left x"))

    def start(self):
        self.plotter.start_time(self.start_time)

    def close(self):
        self.plotter.close()


Runner().run()
