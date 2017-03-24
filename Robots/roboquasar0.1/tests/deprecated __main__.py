import argparse
import math

from actuators.brakes import Brakes
from actuators.steering import Steering
from algorithms.kalman.kalman_constants import constants
from algorithms.kalman.kalman_filter import GrovesKalmanFilter, get_gps_orientation
from algorithms.slam.lidar_slam import SLAM
from atlasbuggy.interface.live import RobotRunner
from atlasbuggy.interface.simulated import RobotSimulator
from atlasbuggy.plotters.liveplotter import LivePlotter
from atlasbuggy.plotters.staticplotter import StaticPlotter
from joysticks.wiiu_joystick import WiiUJoystick
from sensors.gps import GPS
from sensors.imu import IMU
from sensors.lidarturret import LidarTurret

from actuators.underglow import Underglow

from plotters.kalman_plots import KalmanPlots
from algorithms.slam.slam_plots import SlamPlots

parser = argparse.ArgumentParser()
parser.add_argument("-s", "--simulate", help="run in simulated mode", action="store_true")
parser.add_argument("-n", "--lognum", help="file set to use for plotting", nargs=1)
parser.add_argument("-v", "--verbose", help="print all actions of the robot",
                    action="store_true")
parser.add_argument("-l", "--log", help="enable data logging",
                    action="store_true")
parser.add_argument("-a", "--animate", help="enable live plots",
                    action="store_true")
parser.add_argument("-li", "--onlylidar", help="only display lidar data",
                    action="store_true")
parser.add_argument("-k", "--kalman", help="run kalman live",
                    action="store_true")
parser.add_argument("-sl", "--computeslam", help="run SLAM live",
                    action="store_true")
parser.add_argument("-inl", "--includelidar", help="include lidar data in run",
                    action="store_false")
parser.add_argument("-who", "--whoareyou", help="run whoareyou utility",
                    action="store_true")

args = parser.parse_args()

simulated = args.simulate
animate = args.animate

if args.computeslam or not args.includelidar:
    only_lidar = False
else:
    only_lidar = args.onlylidar

if only_lidar:
    animate = True
    use_filter = False
else:
    if args.computeslam:
        use_filter = True
        animate = True
    else:
        use_filter = args.kalman
        animate = args.animate

plots_enabled = animate or simulated

if not args.whoareyou:
    print("using simulator" if simulated else "running live!")
    print("no plots\n" if not plots_enabled else "", end="")
    print("logging data\n" if args.log and not args.simulate else "", end="")
    print("using filter" if use_filter else "not using filter")
    print("only displaying lidar\n" if only_lidar else "", end="")
    print("excluding lidar\n" if args.includelidar else "", end="")
    print("animating data" if animate else "using static plot")
    print("computing and displaying SLAM\n" if args.computeslam else "", end="")

if simulated:
    Interface = RobotSimulator
else:
    Interface = RobotRunner


class RoboQuasar(Interface):
    def __init__(self, set_num=-1):
        self.gps = GPS()
        self.imu = IMU()
        self.turret = LidarTurret()

        self.steering = Steering()
        self.brakes = Brakes()
        self.underglow = Underglow()

        joystick = None

        if only_lidar:
            self.gps.enabled = False
            self.imu.enabled = False
            self.steering.enabled = False
            self.brakes.enabled = False
            self.underglow.enabled = False
        elif not simulated:
            joystick = WiiUJoystick()

        if args.includelidar:
            self.turret.enabled = False

        # self.imu_plot = RobotPlot("Magnetometer data (x, z vs. time)", flat_plot=True, skip_count=20,
        #                           plot_enabled=False)
        self.kalman_plots = KalmanPlots(not only_lidar, use_filter, animate)

        if animate:
            if args.includelidar:
                self.turret.point_cloud_plot.enabled = False
            self.live_plot = LivePlotter(
                2,
                self.kalman_plots.filter_comparison,
                self.turret.point_cloud_plot,
                skip_count=10)

            if self.turret.point_cloud_plot.enabled:
                self.live_plot.draw_dot(self.turret.point_cloud_plot, 0, 0, color='orange', markersize=5)
        elif plots_enabled:
            self.static_plot = StaticPlotter(1, self.kalman_plots.filter_comparison)

        self.filter = None

        # m = np.array(MapFile("buggy course checkpoints").map)
        # self.map_plot.update(m[:, 0], m[:, 1])

        file_sets = (
            ("16;49", "2017_Feb_17"),  # default, latest

            # data day 4
            # ("15", "data_days/2017_Feb_14"),  # filter explodes, LIDAR interfered by the sun
            # ("16;20", "data_days/2017_Feb_14"),  # shorten run, LIDAR collapsed
            # ("16;57", "data_days/2017_Feb_14"),  # interfered LIDAR
            # ("17;10", "data_days/2017_Feb_14"),  # all data is fine, interfered LIDAR
            # ("17;33", "data_days/2017_Feb_14"),  # data is fine, normal run

            # data day 3
            # ("16;38", "data_days/2017_Feb_08"),
            # ("17", "data_days/2017_Feb_08"),
            # ("18", "data_days/2017_Feb_08"),

            # trackfield no gyro values
            # ("15;46", "old_data/2016_Dec_02"),
            # ("15;54", "old_data/2016_Dec_02"),
            # ("16;10", "old_data/2016_Dec_02"),
            # ("16;10", "old_data/2016_Dec_02")

            # rolls day 1
            # ("07;22", "old_data/2016_Nov_06"),  # bad gyro values

            # rolls day 2
            # ("07;36;03 m", "old_data/2016_Nov_12"),
            # ("09;12", "old_data/2016_Nov_12"),  # invalid values
            # ("07;04;57", "old_data/2016_Nov_13"),  # bad gyro values

            # rolls day 3
            # ("modified 07;04", "old_data/2016_Nov_13"),
            # ("modified 07;23", "old_data/2016_Nov_13"),  # wonky value for mag.

            # rolling on the cut
            # ("16;29", "old_data/2016_Dec_09"),
            # ("16;49", "old_data/2016_Dec_09"),
            # ("16;5", "old_data/2016_Dec_09"),
            # ("17;", "old_data/2016_Dec_09"),  # nothing wrong, really short

            # bad data
            # ("16;07", "old_data/2016_Dec_09/bad_data"),  # nothing wrong, really short
            # ("16;09", "old_data/2016_Dec_09/bad_data"),  # nothing wrong, really short
            # ("18;00", "old_data/2016_Dec_09/bad_data"),  # gps spazzed out
            # ("18;02", "old_data/2016_Dec_09/bad_data"),  # gps spazzed out
            # ("18;09", "old_data/2016_Dec_09/bad_data"),  # gps spazzed out
        )

        self.checkpoint_num = 0

        self.lat1, self.long1, self.alt1 = 0, 0, 0
        self.lat2, self.long2, self.alt2 = 0, 0, 0
        self.num_recv_from_fix = None
        self.received_fix = False

        self.imu_t0 = 0
        self.gps_t0 = 0
        self.lidar_t0 = 0

        if simulated:
            super(RoboQuasar, self).__init__(
                file_sets[set_num][0], file_sets[set_num][1],
                self.gps, self.imu,
                self.turret,
                start_index=10000,
                # end_index=2000,
            )

            extension_index = self.parser.full_path.rfind(".")
            image_path = self.parser.full_path[:extension_index]
        else:
            super(RoboQuasar, self).__init__(
                self.imu,
                self.gps,
                self.steering,
                self.brakes,
                self.underglow,
                self.turret,

                joystick=joystick,
                debug_prints=args.verbose,
                log_data=args.log
            )

            extension_index = self.logger.full_path.rfind(".")
            image_path = self.logger.full_path[:extension_index]

        self.slam = SLAM(self.turret, image_path + " post map")
        self.slam_plots = SlamPlots(self.slam.map_size_pixels, self.slam.map_size_meters, args.computeslam)

    def init_filter(self, timestamp):
        self.lat2, self.long2, self.alt2 = self.gps.latitude_deg, self.gps.longitude_deg, self.gps.altitude
        roll, pitch, yaw = get_gps_orientation(self.lat1, self.long1, self.alt1,
                                               self.lat2, self.long2, self.alt2)

        yaw = math.atan2(self.lat2 - self.lat1, self.long2 - self.long1)
        print(math.degrees(roll), math.degrees(pitch), math.degrees(yaw))

        self.filter = GrovesKalmanFilter(
            initial_roll=roll,
            initial_pitch=pitch,
            initial_yaw=yaw,
            initial_lat=self.lat2,
            initial_long=self.long2,
            initial_alt=self.alt2,
            **constants
        )

        self.imu_t0 = timestamp
        self.gps_t0 = timestamp
        self.lidar_t0 = timestamp

        self.kalman_plots.update_gps(self.gps.latitude_deg, self.gps.longitude_deg, self.gps.altitude)
        self.kalman_plots.update_kalman(self.filter.get_position())

        # length = math.sqrt(lat1 ** 2 + long1 ** 2)
        lat2 = 0.0003 * math.sin(yaw) + self.lat1
        long2 = 0.0003 * math.cos(yaw) + self.long1
        self.kalman_plots.initial_compass_plot.updated([self.lat1, lat2], [self.long1, long2])

    def gps_in_range(self):
        if not 280 < self.gps.altitude < 310:
            self.gps.altitude = 300.0
        if -80 < self.gps.longitude_deg < -79.8 and 40.4 < self.gps.latitude_deg < 41 and (
                            280 < self.gps.altitude < 310 or self.gps.altitude == 0.0):
            return True
        else:
            return False

    def update_gps_plot(self):
        if self.gps_in_range():
            self.kalman_plots.update_gps(self.gps.latitude_deg, self.gps.longitude_deg, self.gps.altitude)
            return True
        else:
            return False

    def update_epoch(self, timestamp):
        if use_filter:
            if self.gps.fix and self.gps.latitude is not None and self.received_fix != self.gps.fix:
                self.received_fix = True
                self.num_recv_from_fix = self.num_received(self.gps)

            if self.num_recv_from_fix is not None and self.num_received(self.gps) - self.num_recv_from_fix == 245:
                self.lat1, self.long1, self.alt1 = self.gps.latitude_deg, self.gps.longitude_deg, self.gps.altitude

            elif self.filter is None and self.num_recv_from_fix is not None and \
                    self.gps_in_range() and self.num_received(self.gps) - self.num_recv_from_fix >= 250:
                if use_filter:
                    self.init_filter(timestamp)

            elif self.filter is not None and self.num_recv_from_fix is not None:
                # if self.num_received(self.gps) % 5 == 0:
                if self.update_gps_plot():
                    if use_filter:
                        self.filter.gps_updated(timestamp - self.gps_t0,
                                                self.gps.latitude_deg, self.gps.longitude_deg, self.gps.altitude)
                        self.kalman_plots.update_kalman(self.filter.get_position())
                        self.gps_t0 = timestamp

                        lat1, long1, alt1 = self.filter.get_position()
                        roll, pitch, yaw = self.filter.get_orientation()

                        yaw = -self.imu.euler.x
                        # declination = 9, 19
                        #
                        # self.declination = \
                        #     (declination[0] + declination[1] / 60) * math.pi / 180
                        #
                        # yaw = math.atan2(self.imu.mag.y, self.imu.mag.x)
                        # yaw += self.declination
                        # # Correct for reversed heading
                        # if yaw < 0:
                        #     yaw += 2 * math.pi
                        # # Check for wrap and compensate
                        # elif yaw > 2 * math.pi:
                        #     yaw -= 2 * math.pi

                        # length = math.sqrt(lat1 ** 2 + long1 ** 2)
                        lat2 = 0.0003 * math.sin(math.radians(yaw)) + lat1
                        long2 = 0.0003 * math.cos(math.radians(yaw)) + long1

                        self.kalman_plots.update_compass(lat1, lat2, long1, long2)

                        # print(self.gps.latitude_deg, self.gps.longitude_deg, self.gps.altitude)
                        # if use_filter:
                        # print(self.filter.get_position(), self.filter.properties.estimated_velocity.T.tolist())
                        # print(math.degrees(roll), math.degrees(pitch), math.degrees(yaw))

                    if animate and self.live_plot.should_update():
                        if self.live_plot.plot() is False:
                            return False

    def update_ins(self, timestamp):
        if use_filter and self.filter is not None and self.imu_t0 != timestamp:
            self.filter.imu_updated(timestamp - self.imu_t0,
                                    self.imu.linaccel.x, self.imu.linaccel.y, self.imu.linaccel.z,
                                    self.imu.gyro.x, self.imu.gyro.y, self.imu.gyro.z)
            self.kalman_plots.update_kalman(self.filter.get_position())

            self.imu_t0 = timestamp

    def update_turret(self, timestamp):
        if not args.includelidar and self.turret.did_cloud_update():
            if self.turret.is_cloud_ready() and self.filter is not None and (
                            self.num_received(self.gps) - self.num_recv_from_fix > 250):
                v = self.filter.get_velocity()
                vx = v[0]
                vy = v[1]

                ang_v = self.filter.get_angular(timestamp - self.lidar_t0)[2]
                self.slam.update(timestamp, self.turret.distances, [vx, vy, ang_v])
                self.lidar_t0 = timestamp
                if animate:
                    self.slam_plots.update(self.slam.mapbytes, self.slam.get_pos())

                    # print(self.slam.get_pos(), (vx, vy, ang_v))

            if animate and not self.live_plot.plot():
                return False

    # ----- live methods -----
    def packet_received(self, timestamp, whoiam, packet):
        if self.did_receive(self.imu):
            if self.update_ins(timestamp) is False:
                return False
        elif self.did_receive(self.gps):
            if self.update_epoch(timestamp) is False:
                return False
            print(timestamp)
            print(self.gps)
            print(self.imu)
        elif self.did_receive(self.turret):
            if self.update_turret(timestamp) is False:
                return False
        else:
            print(packet)
                # elif self.did_receive(self.steering):# and self.steering.goal_reached:
                # print(self.steering.current_step)

    def loop(self):
        if not simulated:
            if self.joystick is not None:
                if self.steering.calibrated:
                    if self.joystick.axis_updated("left x"):
                        self.steering.set_speed(self.joystick.get_axis("left x"))
                    elif self.joystick.button_updated("A") and self.joystick.get_button("A"):
                        self.steering.set_position(0)

                if self.joystick.button_updated("B") and self.joystick.get_button("B"):
                    self.brakes.pull()
                elif self.joystick.button_updated("X") and self.joystick.get_button("X"):
                    self.brakes.release()
                elif self.joystick.button_updated("Y") and self.joystick.get_button("Y"):
                    self.record("checkpoint", "%s\t%s\t%s" % (self.checkpoint_num, self.gps.latitude_deg, self.gps.longitude_deg))
                    if self.gps.latitude_deg is not None and self.gps.longitude_deg is not None:
                        print("checkpoint #%s: %3.6f, %3.6f" % (self.checkpoint_num, self.gps.latitude_deg, self.gps.longitude_deg))
                    self.checkpoint_num += 1
                    print("checkpoint #%s" % self.checkpoint_num)
                    # elif self.joystick.dpad_updated():
                    #     if self.joystick.dpad[1] == 1:
                    #         self.brakes.set_brake(self.brakes.goal_position + 20)
                    #     elif self.joystick.dpad[1] == -1:
                    #         self.brakes.set_brake(self.brakes.goal_position - 20)

    def start(self):
        if animate:
            self.live_plot.start_time(self.start_time)

    # ----- simulator methods -----

    def object_packet(self, timestamp):
        if self.did_receive(self.imu):
            if self.update_ins(timestamp) is False:
                return False
        elif self.did_receive(self.gps):
            if self.update_epoch(timestamp) is False:
                return False
        elif self.did_receive(self.turret):
            if self.update_turret(timestamp) is False:
                return False

    # def user_packet(self, timestamp, packet):
        # if self.did_receive("checkpoint"):
        #     print(timestamp, packet)

    # def command_packet(self, timestamp, packet):
    #     if self.did_receive(self.steering):
    #         print(timestamp, packet)

    def close(self):
        if plots_enabled:
            if not animate:
                self.static_plot.plot()
                self.static_plot.show()
            else:
                if simulated:
                    if not self.error_signalled:
                        print("Finished!")
                        self.live_plot.freeze_plot()

                self.live_plot.close()

        if args.computeslam:
            self.slam.make_image()


if args.whoareyou:
    from threading import Thread
    import serial.tools.list_ports

    from atlasbuggy.robot.port import RobotSerialPort

    if len(serial.tools.list_ports.comports()) == 0:
        print("No ports found!")

    message = ""


    def discover_port(port_info):
        global message
        if port_info.serial_number is not None:
            robot_port = RobotSerialPort(port_info, True, None, None, None, None)
            if not robot_port.configured:
                # robot_port.stop()
                # raise RobotSerialPortNotConfiguredError("Port not configured!", robot_port)
                robot_port.stop()
                # pprint.pprint(port_info.__dict__)
                # print("-----------------------------------")
                # print("address '%s' does not abide atlasbuggy protocol!" % (port_info.device))
                # print("-----------------------------------")
                message += "address '%s' does not abide atlasbuggy protocol!\n" % (port_info.device)
            else:
                message += "address '%s' has ID '%s'\n" % (
                    port_info.device, robot_port.whoiam if robot_port.whoiam is not None else '')
                robot_port.stop()


    threads = []
    for port_info in serial.tools.list_ports.comports():
        thread = Thread(target=discover_port, args=(port_info,))
        threads.append(thread)
        thread.start()

    for thread in threads:
        thread.join()

    print()
    print(message)

else:
    if not simulated or args.lognum[0] == "last":
        RoboQuasar().run()
    else:
        RoboQuasar(int(args.lognum[0])).run()
