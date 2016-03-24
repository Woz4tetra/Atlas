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
from analyzers.interpreter import Interpreter
from analyzers.kalman_filter import PositionFilter, HeadingFilter
from analyzers.logger import Recorder
from controllers.gcjoystick import joystick_init
from controllers.servo_map import *
from microcontroller.data import Command
from microcontroller.data import Sensor
from microcontroller.dashboard import start, stop, is_running, reset
from sound.player import TunePlayer


def main(log_data=True, manual_mode=False, print_data=True):
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
        log = Recorder(
            'gps lat', 'gps long', 'gps heading', 'gps found', 'gps flag',
            'gps sleep time',
            'encoder counts', 'encoder flag', 'encoder sleep time',
            'imu accel x', 'imu accel y', 'imu yaw', 'imu compass', 'imu flag',
            'imu sleep time',
            'kalman x', 'kalman y', 'kalman heading', 'goal x', 'goal y',
            'servo',
            directory="Autonomous Test Day 2", frequency=0.01)

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

            if is_running() != prev_status:
                if is_running():
                    print("Connection made!")
                    notifier.play("short victory")
                else:
                    print("Connection lost...")
                    notifier.play("broken")
                prev_status = is_running()

            kalman_heading = heading_filter.update(imu["compass"], imu_flag,
                                                   gps["heading"], gps_flag)

            gps_x, gps_y, accel_x, accel_y, change_dist, heading = \
                interpreter.convert(
                    gps["lat"], gps["long"], imu["accel_x"], imu["accel_y"],
                    encoder["counts"], imu["yaw"], kalman_heading
                )

            current_time = time.time()

            kalman_x, kalman_y = position_filter.update(
                gps_x, gps_y, gps_flag,
                accel_x, accel_y, heading, imu_flag,
                change_dist, enc_flag, encoder.sleep_time,
                current_time - prev_time
            )

            prev_time = current_time

            goal_x, goal_y = binder.bind((kalman_x, kalman_y))

            if print_data:
                if imu_flag:
                    print(("%0.4f\t" * 5) % (
                        time.time() - prev_time,
                        imu["accel_x"], imu["accel_y"],
                        imu["compass"], imu["yaw"]))
                if gps_flag:
                    print(time.time() - prev_time, gps["lat"], gps["long"],
                          gps["heading"], gps["found"])
                if enc_flag:
                    print(time.time() - prev_time, encoder["counts"])

            if manual_mode:
                servo_steering["position"] = \
                    servo_value([0, 0, 0],
                                [1, 5.34 / 90 * joystick.mainStick.x])
                print(time.time() - prev_time, servo_steering["position"])
            else:
                servo_steering["position"] = \
                    servo_value((kalman_x, kalman_y, kalman_heading),
                                (goal_x, goal_y))

            if log_data:
                log.add_row(gps['lat'], gps['long'], gps['heading'],
                            gps['found'], gps_flag, gps.sleep_time,
                            encoder['counts'], enc_flag, encoder.sleep_time,
                            imu['accel_x'], imu['accel_y'], imu['yaw'],
                            imu['compass'], imu_flag, imu.sleep_time,
                            kalman_x, kalman_y, kalman_heading, goal_x, goal_y,
                            servo_steering["position"])

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
