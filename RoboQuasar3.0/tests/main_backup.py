"""
Written by Ben Warwick

RoboQuasar3.0, written for the Atlas Project (autonomous buggy group)
Version 3/16/2016
=========

Usage
-----
python __main__.py
- or - (in project's parent directory):
python RoboQuasar3.0

"""
import time
import traceback

from analyzers.binder import Binder
from analyzers.converter import Interpreter
from analyzers.kalman_filter import PositionFilter, HeadingFilter
from analyzers.logger import Recorder
from controllers.joystick import joystick_init
from controllers.servo_map import *
from microcontroller.data import Command
from microcontroller.data import Sensor
from microcontroller.data import start, stop, is_running, reset
from sound.player import TunePlayer


# Binder and/or the map (more likely, set a point off in the distance) might be fucked
# shifting needs to be done during a run (localize heading)
# make sure angle shifting is consistent

def main(log_data=False, manual_mode=False, print_data=True):
    # data type is specified by incoming packet
    gps = Sensor(1, ['lat', 'long', 'heading', 'found'])
    encoder = Sensor(2, 'counts')
    imu = Sensor(3, ['accel_x', 'accel_y', 'yaw', 'compass'])

    servo_steering = Command(0, 'position', (-90, 90))
    # servo_brakes = Command(1, 'position', (-90, 90))

    joystick = joystick_init()
    notifier = TunePlayer()

    start(use_handshake=False)

    print("Wait for the GPS to lock on, then press A")

    while not gps['found']:
        joystick.update()
        time.sleep(0.005)
    print(gps['lat'], gps['long'])
    notifier.play("bloop")
    while not joystick.buttons.A:
        joystick.update()

    reset()

    notifier.play("ding")
    interpreter = Interpreter(0, gps["lat"], gps["long"])
    position_filter = PositionFilter()
    heading_filter = HeadingFilter()
    binder = Binder("Track Field Map Trimmed.csv")

    log = None

    prev_status = not is_running()
    prev_gps_status = not gps['found']

    time.sleep(0.5)

    prev_time = time.time()

    if log_data:
        # TODO: make enable flags per sensor based
        log = Recorder(directory="Autonomous Test Day 2", frequency=0.01)
        log.add_tracker(imu, 'imu')
        log.add_tracker(encoder, 'encoder')
        log.add_tracker(gps, 'gps')
        log.add_tracker(servo_steering, 'servo_steering')
        # log.add_tracker(servo_brakes, 'servo_brakes')
        log.end_init()

    try:
        while True:  # maybe in future, reset global events here (received data)
            joystick.update()
            if joystick.buttons.B:
                manual_mode = not manual_mode
                if manual_mode:
                    notifier.play("got thing")
                else:
                    notifier.play("ring")
                while joystick.buttons.B:
                    joystick.update()

            imu_flag = imu.received()
            gps_flag = gps.received()
            enc_flag = encoder.received()

            if gps_flag:
                notifier.play("click")

            if prev_gps_status != gps['found']:
                if gps['found']:
                    notifier.play("short victory")
                else:
                    notifier.play("saved")
                prev_gps_status = gps['found']

            if print_data:
                # if imu_flag:
                #     print(("%0.4f\t" * 5) % (
                #         time.time() - prev_time,
                #         imu["accel_x"], imu["accel_y"],
                #         imu["compass"], imu["yaw"]))
                # if gps_flag:
                #     print(time.time() - prev_time, gps["lat"], gps["long"],
                #           gps["heading"], gps["found"])
                # if enc_flag:
                #     print(time.time() - prev_time, encoder["counts"])
                print(("%0.4f\t" * 5 + "%i\t" + "%0.4f\t" * 4 + "%i\t" * 2 + "%0.4f\t" + "%i\t%i\t%0.4f") % (
                    time.time() - prev_time,
                    imu["accel_x"], imu["accel_y"],
                    imu["compass"], imu["yaw"], imu_flag, imu.sleep_time,
                    gps["lat"], gps["long"], gps["heading"], gps["found"], gps_flag, gps.sleep_time,
                    encoder["counts"], enc_flag, encoder.sleep_time))

            if is_running() != prev_status:
                if is_running():
                    print("Connection made!")
                    notifier.play("short victory")
                else:
                    print("Connection lost...")
                    notifier.play("broken")
                prev_status = is_running()

            if manual_mode:
                # servo_steering["position"] = int(
                #     50 * (joystick.triggers.L - joystick.triggers.R)) - 23
                servo_steering["position"] = \
                    state_to_servo([0, 0, 0],
                                   [1, 5.34 / 90 * joystick.mainStick.x])
                print(time.time() - prev_time, servo_steering["position"])
            else:
                current_time = time.time()

                kalman_heading = heading_filter.update(imu["compass"], imu_flag,
                                                       gps["heading"], gps_flag)

                gps_x, gps_y, accel_x, accel_y, change_dist, heading = \
                    interpreter.convert(
                        gps["lat"], gps["long"], imu["accel_x"], imu["accel_y"],
                        encoder["counts"], imu["yaw"], kalman_heading
                    )

                x, y = position_filter.update(
                    gps_x, gps_y, gps_flag,
                    accel_x, accel_y, heading, imu_flag,
                    change_dist, enc_flag, encoder.sleep_time,
                    current_time - prev_time
                )

                goal_x, goal_y = binder.bind((x, y))

                servo_steering["position"] = \
                    state_to_servo((x, y, kalman_heading), (goal_x, goal_y))

                print(goal_x, goal_y, x, y, heading, kalman_heading,
                      servo_steering["position"])

                prev_time = current_time

            if log_data:
                log.add_data(imu, imu_flag)
                log.add_data(encoder, gps_flag)
                log.add_data(gps, enc_flag)
                log.add_data(servo_steering, True)
                # log.add_data(servo_brakes)
                log.end_row()

            time.sleep(0.001)

    except KeyboardInterrupt:
        traceback.print_exc()
    finally:
        stop()
        notifier.play("PuzzleDone")
        time.sleep(1)


if __name__ == '__main__':
    print(__doc__)
    main(log_data=True, manual_mode=False, print_data=True)
