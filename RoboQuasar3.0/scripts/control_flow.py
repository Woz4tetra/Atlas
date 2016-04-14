"""
Written by Ben Warwick

RoboQuasar3.0, written for the Atlas Project (autonomous buggy group)
Version 4/2/2016
=========

Usage
-----
python trackfield_test4.py

"""

import sys
import time
import traceback

sys.path.insert(0, "../")

from analyzers.binder import Binder
from analyzers.converter import HeadingConverter, PositionConverter
from analyzers.logger import Recorder
from analyzers.kalman_filter import HeadingFilter, PositionFilter
from controllers.joystick import joystick_init
from microcontroller.data import Command
from microcontroller.data import Sensor
from microcontroller.dashboard import start, stop, is_running, reset
from controllers.servo_map import state_to_servo
from sound.player import TunePlayer


def main(log_data=True, manual_mode=True, print_data=True):
    print("log_data = %s, manual_mode = %s, print_data = %s" %
          (log_data, manual_mode, print_data))

    gps = Sensor(1, ['lat', 'long', 'heading', 'found'])
    encoder = Sensor(2, 'counts')
    imu = Sensor(3, ['accel_x', 'accel_y', 'yaw'])

    servo_steering = Command(0, 'position', (-90, 90))

    joystick = joystick_init()
    notifier = TunePlayer()

    start(use_handshake=False)

    print("Wait for the GPS to lock on, then press A")

    while not gps['found'] and not joystick.buttons.A:
        joystick.update()
        time.sleep(0.005)
    print(gps['lat'], gps['long'])
    notifier.play("bloop")
    time.sleep(0.05)
    while not joystick.buttons.A:
        joystick.update()
        time.sleep(0.005)

    reset()

    notifier.play("ding")
    binder = Binder("Track Field Map Trimmed.csv")
    heading_converter = HeadingConverter(gps["lat"], gps["long"])
    position_converter = PositionConverter(
        encoder["counts"], gps["lat"], gps["long"])
    position_filter = PositionFilter()
    heading_filter = HeadingFilter()

    log = None

    prev_status = is_running()
    prev_gps_status = gps['found']

    time.sleep(0.5)

    prev_time = time.time()

    bind_x, bind_y = 0, 0
    bind_flag = False

    imu_flag = False
    gps_flag = False
    enc_flag = False

    if log_data:
        log = Recorder(directory="Test Day 5", headers=["gps_lat", "gps_long",
        "gps_heading",
        "gps_found",
        "gps_flag", "gps_sleep",
        "encoder", "enc_flag",
        "enc_sleep",
        "accel_x", "accel_y",
        'yaw', "imu_flag",
        "imu_sleep",
        "kalman_x", "kalman_y",
        "kalman_heading",
        "bind_x", "bind_y", "servo_steering"])#, frequency=0.01)

    try:
        while True:
            joystick.update()
            if joystick.buttons.B:
                break

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

            gps_heading, bind_heading = heading_converter.convert(
                gps['long'], gps['lat'], bind_x, bind_y
            )

            kalman_heading = heading_filter.update(
                gps_heading, gps_flag, bind_heading, bind_flag,
                imu["yaw"], imu_flag
            )
            #what does this do? line below. NOTE
            if not bind_flag:
                bind_flag = True

            gps_x, gps_y, shifted_ax, shifted_ay, enc_dist = \
                position_converter.convert(
                    gps["long"], gps["lat"], imu["accel_x"], imu["accel_y"],
                    encoder["counts"], kalman_heading
                )

            kalman_x, kalman_y = position_filter.update(
                gps_x, gps_y, gps_flag, gps.sleep_time,
                shifted_ax, shifted_ay, imu_flag, imu.sleep_time,
                enc_dist, enc_flag, encoder.sleep_time, time.time() - prev_time,
                kalman_heading
            )

            prev_time = time.time()

            bind_x, bind_y = binder.bind((kalman_x, kalman_y))

            #NOTE what are the inputs to this
            servo_steering["position"] = \
                state_to_servo([0, 0, 0],
                               [1, 5.34 / 90 * joystick.mainStick.x])

            if log_data:
                # log.add_row(OrderedDict(
                #     gps_lat=gps['lat'], gps_long=gps['long'],
                #     gps_heading=gps['heading'],
                #     gps_found=gps['found'],
                #     gps_flag=gps_flag, gps_sleep=gps.sleep_time,
                #     encoder=encoder['counts'], enc_flag=enc_flag,
                #     enc_sleep=encoder.sleep_time,
                #     accel_x=imu['accel_x'], accel_y=imu['accel_y'],
                #     yaw=imu['yaw'], imu_flag=imu_flag,
                #     imu_sleep=imu.sleep_time,
                #     kalman_x=kalman_x, kalman_y=kalman_y,
                #     kalman_heading=kalman_heading,
                #     bind_x=bind_x, bind_y=bind_y,
                #     servo_steering=servo_steering["position"]
                # ))
                log.add_row(
                    [gps['lat'], gps['long'],
                    gps['heading'],
                    gps['found'],
                    gps_flag, gps.sleep_time,
                    encoder['counts'], enc_flag,
                    encoder.sleep_time,
                    imu['accel_x'], imu['accel_y'],
                    imu['yaw'], imu_flag,
                    imu.sleep_time,
                    kalman_x, kalman_y,
                    kalman_heading,
                    bind_x, bind_y,
                    servo_steering["position"]]
                )
    except KeyboardInterrupt:
        traceback.print_exc()
    finally:
        stop()
        joystick.stop()
        log.close()
        # notifier.play("PuzzleDone")
        time.sleep(1)


if __name__ == '__main__':
    print(__doc__)
    main()
