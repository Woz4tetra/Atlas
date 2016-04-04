"""
Written by Ben Warwick

RoboQuasar3.0, written for the Atlas Project (autonomous buggy group)
Version 4/2/2016
=========

Usage
-----
python trackfield_test4.py

"""

from analyzers.binder import Binder
from analyzers.interpreter import Interpreter
from analyzers.logger import Recorder
from analyzers.fancy_kalman import KalmanFilter
from analyzers.heading_filter import HeadingFilter
from controllers.gcjoystick import joystick_init
from controllers.servo_map import *
from microcontroller.data import Command
from microcontroller.data import Sensor
from microcontroller.dashboard import start, stop, is_running, reset
from sound.player import TunePlayer

def main(log_data=True, manual_mode=False, print_data=True):
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

    while not gps['found']:
        time.sleep(0.005)
    print(gps['lat'], gps['long'])
    notifier.play("bloop")
    while not joystick.buttons.A:
        joystick.update()
        time.sleep(0.005)

    reset()

    notifier.play("ding")
    binder = Binder("Track Field Map Trimmed.csv")
    interpreter = Interpreter(0, gps["lat"], gps["long"])
    position_filter = KalmanFilter()
    heading_filter = HeadingFilter()

    log = None

    prev_status = not is_running()
    prev_gps_status = not gps['found']

    time.sleep(0.5)

    prev_time = time.time()

    if log_data:
        log = Recorder(directory="Autonomous Test Day 3", frequency=0.01)

    bound_x, bound_y = 0, 0
    bind_flag = False

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

            heading = heading_filter.update(
                gps["heading"], gps_flag,
                (bound_x, bound_y), bind_flag,
                imu["yaw"], imu_flag
            )

            gps_x, gps_y, accel_x, accel_y, change_dist = \
                interpreter.convert(
                    gps["lat"], gps["long"], imu["accel_x"], imu["accel_y"],
                    encoder["counts"], imu["yaw"], heading
                )

            current_time = time.time()  # TODO: is this the correct place for this?

            x, y = position_filter.update(
                gps_x, gps_y, gps_flag, gps.sleep_time,
                accel_x, accel_y, imu_flag, imu.sleep_time,
                change_dist, enc_flag, encoder.sleep_time,
                current_time - prev_time, heading
            )

            prev_time = current_time   # TODO: is this the correct place for this?

            bound_x, bound_y = binder.bind((x, y))

            if log_data:
                log.add_row(
                    gps_lat=gps['lat'], gps_long=gps['long'],
                    gps_heading=gps['heading'],
                    gps_found=gps['found'],
                    gps_flag=gps_flag, gps_sleep=gps.sleep_time,
                    encoder=encoder['counts'], enc_flag=enc_flag,
                    enc_sleep=encoder.sleep_time,
                    accel_x=imu['accel_x'], accel_y=imu['accel_y'],
                    yaw=imu['yaw'], imu_flag=imu_flag,
                    imu_sleep=imu.sleep_time,
                    kalman_x=x, kalman_y=y, kalman_heading=heading,
                    bound_x=bound_x, bound_y=bound_y,
                    servo_steering=servo_steering["position"]
                )
            bind_flag = True

if __name__ == '__main__':
    print(__doc__)
    main()
