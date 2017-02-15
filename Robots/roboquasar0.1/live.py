import sys
from atlasbuggy.robot.interface import RobotInterface

from joysticks.wiiu_joystick import WiiUJoystick

from sensors.gps import GPS
from sensors.imu import IMU
from sensors.lidarturret import LidarTurret
from actuators.steering import Steering
from actuators.brakes import Brakes
from actuators.underglow import Underglow

live_plotting = False

if live_plotting:
    from atlasbuggy.plotters.liveplotter import LivePlotter
    from atlasbuggy.plotters.robotplot import RobotPlot


class Runner(RobotInterface):
    def __init__(self):
        self.gps = GPS()
        self.imu = IMU()
        self.steering = Steering()
        self.brakes = Brakes()
        self.underglow = Underglow(enabled=False)
        self.turret = LidarTurret(enabled=False)

        if live_plotting:
            self.plotter = LivePlotter(1, self.turret.point_cloud_plot)
            self.plotter.draw_dot("point cloud", 0, 0, color='orange', markersize=5)

        if len(sys.argv) > 1 and sys.argv[1] == "nolog":
            log_data = False
        else:
            log_data = True

        super(Runner, self).__init__(
            self.imu,
            self.gps,
            self.steering,
            self.brakes,
            self.underglow,
            self.turret,

            joystick=WiiUJoystick(),
            debug_prints=True,
            log_data=log_data
        )

    def packet_received(self, timestamp, whoiam, packet):
        if self.did_receive(self.imu):
            if live_plotting:
                if self.turret.did_cloud_update() and self.queue_len() < 25:
                    # if self.turret.is_cloud_ready():
                    if self.plotter.plot() is False:
                        return False
                        # else:
                        #     print(timestamp, self.imu.eul_x, self.imu.accel_x, self.imu.gyro_x, self.imu.mag_x)
                        # self.underglow.set_cycle_val(self.imu.euler.x / 45)
        elif self.did_receive(self.gps):
            print(timestamp)
            print(self.gps)
            print(self.imu)
            # elif self.did_receive(self.steering):# and self.steering.goal_reached:
            # print(self.steering.current_step)

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
                # elif self.joystick.dpad_updated():
                #     if self.joystick.dpad[1] == 1:
                #         self.brakes.set_brake(self.brakes.goal_position + 20)
                #     elif self.joystick.dpad[1] == -1:
                #         self.brakes.set_brake(self.brakes.goal_position - 20)

    def start(self):
        # self.change_port_rate(self.gps, self.gps.baud_rate)
        if live_plotting:
            self.plotter.start_time(self.start_time)

    def close(self):
        if live_plotting:
            self.plotter.close()


Runner().run()
