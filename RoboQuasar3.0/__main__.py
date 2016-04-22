"""
Written by Ben Warwick

RoboQuasar3.0, written for the Atlas Project (autonomous buggy group)
Version 4/22/2016
=========

Usage
-----
python __main__.py
- or - (in project's parent directory):
python RoboQuasar3.0

"""
import traceback

from microcontroller.data import *
from microcontroller.dashboard import *

from analyzers.logger import Recorder
from analyzers.converter import Converter
from analyzers.kalman_filter import HeadingFilter, PositionFilter
from analyzers.binder import Binder

from controllers.joystick import joystick_init
from controllers.servo_map import angle_to_servo, state_to_servo

from sound.player import TunePlayer


class System:
    def __init__(self, log_data, log_dir, map_name):
        self.log_data, = log_data,
        self.log_dir = log_dir
        self.map_name = map_name

        self.binder = None

    def init_state_trackers(self, initial_long, initial_lat, initial_enc):
        self.converter = Converter(initial_long, initial_lat, 0.000003,
                                   initial_enc)

        self.heading_filter = HeadingFilter()
        self.position_filter = PositionFilter()

        if self.binder == None:
            self.binder = Binder(self.map_name, initial_long, initial_lat)
        else:
            self.binder.reshift(initial_long, initial_lat)

        self.prev_time = time.time()

    def start_logging(self):
        if self.log_data:
            self.log = Recorder(directory=self.log_dir)
        else:
            self.log = None

        self.prev_time = time.time()

    def stop_logging(self):
        if self.log_data and self.log is not None:
            self.log.close()

    def update_state(self, gps_long, gps_lat, enc_counts, imu_yaw):
        self.gps_heading, self.bind_heading = self.converter.convert_heading(
            gps_long, gps_lat, self.bind_x, self.bind_y
        )
        self.gps_x, self.gps_y, self.enc_dist = self.converter.convert_position(
            gps_long, gps_lat, enc_counts
        )

        self.kalman_heading = self.heading_filter.update(
            self.gps_heading, self.bind_heading, -imu_yaw
        )
        self.kalman_x, self.kalman_y = self.position_filter.update(
            self.gps_x, self.gps_y, self.enc_dist,
            time.time() - self.prev_time,
            self.kalman_heading
        )
        self.prev_time = time.time()

        self.bind_x, self.bind_y, self.goal_x, self.goal_y = self.binder.bind(
            (self.kalman_x, self.kalman_y))

        return self.goal_x, self.goal_y, self.bind_x, self.bind_y, self.kalman_heading

    def log_state(self, gps, encoder, imu, servo_steering):
        self.log["gps long"] = gps["long"]
        self.log["gps lat"] = gps["lat"]
        self.log["gps found"] = gps["found"]
        self.log["gps sleep"] = gps.sleep_time

        self.log["gps x"] = self.gps_x
        self.log["gps y"] = self.gps_y
        self.log["gps bearing"] = self.gps_heading

        self.log["imu yaw"] = imu["yaw"]
        self.log["imu sleep"] = imu.sleep_time

        self.log["encoder"] = encoder["counts"]
        self.log["encoder sleep"] = encoder.sleep_time
        self.log["encoder dist"] = self.enc_dist

        self.log["bind x"] = self.bind_x
        self.log["bind y"] = self.bind_y

        self.log["goal x"] = self.goal_x
        self.log["goal y"] = self.goal_y

        self.log["bind heading"] = self.bind_heading

        self.log["kalman x"] = self.kalman_x
        self.log["kalman y"] = self.kalman_y
        self.log["kalman heading"] = self.kalman_heading

        self.log["servo"] = servo_steering["position"]

        self.log.add_row()


def main(log_data=True, manual_mode=True, print_data=True):
    # ----- initialize runner -----
    gps = Sensor(1, ['lat', 'long', 'found'])
    encoder = Sensor(2, 'counts')
    imu = Sensor(3, 'yaw')

    servo_steering = Command(0, 'position', (-90, 90))

    joystick = joystick_init()
    notifier = TunePlayer()

    start()

    system = System(log_data, "Test Day 10", "Trimmed Minimalist Map.csv")

    print("Wait for the GPS to lock on, then press A")
    try:
        while not gps['found'] and not joystick.buttons.A:
            time.sleep(0.005)
        print("gps:", gps['lat'], gps['long'])
        notifier.play("bloop")
        time.sleep(0.05)
        while not joystick.buttons.A:
            time.sleep(0.005)

        reset()

        prev_status = is_running()
        prev_gps_status = gps['found']

        # ----- initialize converters and filters -----

        print("Encoder resetting")
        while encoder['counts'] != 0:
            reset()
            time.sleep(0.01)

        system.init_state_trackers(gps['long'], gps['lat'], encoder["counts"])
        system.start_logging()

        notifier.play("ding")

        bind_x, bind_y, goal_x, goal_y, kalman_heading = 0, 0, 0, 0, 0

        # ----- main loop -----
        while True:
            if joystick.buttons.B:
                system.stop_logging()

                notifier.play("ring")

                time.sleep(0.5)

                while not gps['found'] and not joystick.buttons.A:
                    time.sleep(0.005)
                print("gps:", gps['lat'], gps['long'])
                notifier.play("bloop")
                time.sleep(0.05)
                while not joystick.buttons.A:
                    time.sleep(0.005)

                print("Encoder resetting")
                while encoder['counts'] != 0:
                    reset()
                    time.sleep(0.01)

                system.init_state_trackers(gps['long'], gps['lat'],
                                           encoder["counts"])
                system.start_logging()

                notifier.play("ding")

            if joystick.buttons.X:
                while joystick.buttons.X: pass
                notifier.play("ba ding")
                manual_mode = not manual_mode

            # ----- status notifiers -----
            if encoder.received() and print_data:
                print("encoder:", encoder["counts"])
            if prev_gps_status != gps['found']:
                if gps['found']:
                    notifier.play("short victory")
                    print("gps found")
                else:
                    notifier.play("broken")
                    print("gps lost")
                prev_gps_status = gps['found']
            if is_running() != prev_status:
                if is_running():
                    print("Connection made!")
                    notifier.play("short victory")
                else:
                    print("Connection lost...")
                    notifier.play("broken")
                prev_status = is_running()

            # ----- filter data -----
            if gps.received():
                if print_data:
                    print("gps:", gps["long"], gps["lat"], gps["found"])
                notifier.play("click")

                bind_x, bind_y, goal_x, goal_y, kalman_heading = \
                    system.update_state(gps["long"], gps["lat"],
                                        encoder["counts"],
                                        -imu["yaw"])

                if print_data:
                    print("(%0.4f, %0.4f) @ %0.4f -> (%0.4f, %0.4f)" % (
                        bind_x, bind_y, kalman_heading, goal_x, goal_y))

            if manual_mode:
                servo_steering["position"] = \
                    state_to_servo([0, 0, 0],
                                   [1, 5.34 / 90 * joystick.mainStick.x])
            else:
                # servo_steering["position"] = \
                #     state_to_servo([bind_x, bind_y, kalman_heading],
                #                    [goal_x, goal_y])
                servo_steering["position"] = \
                    angle_to_servo(math.atan2(goal_y - bind_y, goal_x - bind_x))
            time.sleep(0.005)

            system.log_state(gps, encoder, imu, servo_steering)
    except KeyboardInterrupt:
        traceback.print_exc()
    finally:
        stop()
        joystick.stop()
        system.stop_logging()


if __name__ == '__main__':
    print(__doc__)
    main()
