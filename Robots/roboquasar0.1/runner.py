from atlasbuggy.robot.interface import RobotInterface

from joysticks.wiiu_joystick import WiiUJoystick

from sensors.gps import GPS
from sensors.imu import IMU
from actuators.steering import Steering
from actuators.brakes import Brakes

live_plotting = False

if live_plotting:
    from atlasbuggy.plotters.liveplotter import LivePlotter
    from atlasbuggy.plotters.robotplot import RobotPlot


class Runner(RobotInterface):
    def __init__(self):
        self.gps = GPS()
        self.imu = IMU()
        self.steering = Steering(enabled=False)
        self.brakes = Brakes()

        if live_plotting:
            self.imu_plot_eul = RobotPlot("imu eul", flat_plot=False, max_length=30)
            self.imu_plot_gyro = RobotPlot("imu gyro", flat_plot=False, max_length=30)
            self.imu_plot_mag = RobotPlot("imu mag", flat_plot=False, max_length=30)
            self.imu_plot_accel = RobotPlot("imu accel", flat_plot=False, max_length=30)

            self.plotter = LivePlotter(2, self.imu_plot_eul, self.imu_plot_gyro, self.imu_plot_mag)

        super(Runner, self).__init__(
            self.imu,
            self.gps,
            self.steering,
            self.brakes,
            # joystick=WiiUJoystick(),
            debug_prints=True,
            log_data=False
        )

    def packet_received(self, timestamp, whoiam, packet):
        if self.did_receive(self.imu):
            if live_plotting:
                if self.queue_len() < 25:
                    self.imu_plot_eul.append(self.imu.eul_x, self.imu.eul_y, self.imu.eul_z)
                    self.imu_plot_mag.append(self.imu.mag_x, self.imu.mag_y, self.imu.mag_z)
                    self.imu_plot_gyro.append(self.imu.gyro_x, self.imu.mag_y, self.imu.mag_z)
                    self.imu_plot_accel.append(self.imu.accel_x, self.imu.accel_y, self.imu.accel_z)
                    if self.plotter.plot() is False:
                        return False
                        # else:
                        #     print(timestamp, self.imu.eul_x, self.imu.accel_x, self.imu.gyro_x, self.imu.mag_x)
        # elif self.did_receive(self.gps):
        #     print(timestamp, self.gps.latitude, self.gps.longitude)
        #     print(timestamp, self.imu.eul_x, self.imu.accel_x, self.imu.gyro_x, self.imu.mag_x)
            # elif self.did_receive(self.steering):# and self.steering.goal_reached:
            # print(self.steering.current_step)

    def loop(self):
        if self.joystick is not None and self.steering.calibrated:
            if self.joystick.axis_updated("left x"):
                self.steering.set_speed(self.joystick.get_axis("left x"))
            elif self.joystick.button_updated("A") and self.joystick.get_button("A"):
                self.steering.set_position(0)
            elif self.joystick.button_updated("B") and self.joystick.get_button("B"):
                self.steering.set_position(200)
            elif self.joystick.button_updated("X") and self.joystick.get_button("X"):


    def start(self):
        self.change_port_rate(self.gps, self.gps.baud_rate)
        if live_plotting:
            self.plotter.start_time(self.start_time)

    def close(self):
        if live_plotting:
            self.plotter.close()


Runner().run()
