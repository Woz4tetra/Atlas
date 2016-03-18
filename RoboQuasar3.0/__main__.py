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
import traceback
import sys
import time
import math

sys.path.insert(0, '../')

from microcontroller.data import Sensor
from microcontroller.data import Command
from microcontroller.data import start, stop, is_running, reset

from analyzers.logger import Recorder
from analyzers.kalman_filter import StateFilter
from analyzers.interpreter import Interpreter
from analyzers.map import Map
from analyzers.binder import Binder

from controllers.gcjoystick import joystick_init
from controllers.servo_map import *

from sound.player import TunePlayer

# Binder and/or the map (more likely, set a point off in the distance) might be fucked
# shifting needs to be done during a run (localize heading)
# make sure angle shifting is consistent

def main(log_data=True, manual_mode=True, print_data=True):
    # data type is specified by incoming packet
    gps = Sensor(1, ['lat', 'long', 'heading'])
    encoder = Sensor(2, 'counts')
    imu = Sensor(3, ['accel_x', 'accel_y', 'yaw', 'compass'])

    servo_steering = Command(0, 'position', (90, -90))
    # servo_brakes = Command(1, 'position', (90, -90))

    joystick = joystick_init()
    notifier = TunePlayer()

    start(use_handshake=False)

    print("Awaiting user input... (Press A)")


    while int(gps['lat']) == 0 or int(gps['long']) == 0:
        joystick.update()
        time.sleep(0.005)
    print(gps['lat'], gps['long'])
    notifier.play("EnterLevel.wav")
    while not joystick.buttons.A:
        joystick.update()

    reset()

    notifier.play("Coin.wav")  # TODO: Find sound effects
    # 1.344451296765884 for shift_angle?
    interpreter = Interpreter(0, gps["lat"], gps["long"], imu["compass"])
    kfilter = StateFilter()
    map = Map("Track Field Map Trimmed.csv",
              origin_lat=gps["lat"],
              origin_long=gps["long"],
              shift_angle=imu["compass"])
    binder = Binder(map)

    log = None

    prev_status = not is_running()

    time.sleep(0.5)

    prev_time = time.time()

    if log_data:
        log = Recorder(frequency=0.01)
        log.add_tracker(imu, 'imu')
        log.add_tracker(encoder, 'encoder')
        log.add_tracker(gps, 'gps')
        log.add_tracker(servo_steering, 'servo_steering')
        # log.add_tracker(servo_brakes, 'servo_brakes')
        log.end_init()

    try:
        while True:
            joystick.update()
            if joystick.buttons.B:
                manual_mode = not manual_mode
                while joystick.buttons.B:
                    joystick.update()

            imu_flag = imu.received()
            gps_flag = gps.received()
            enc_flag = encoder.received()

            # if print_data:
            #     if imu_flag:
            #         print(("%0.4f\t" * 5) % (
            #             time.time() - prev_time,
            #             imu["accel_x"], imu["accel_y"],
            #             imu["compass"], imu["yaw"]))
            #     if gps_flag:
            #         print(time.time() - prev_time, gps["lat"], gps["long"], gps["heading"])
            #         # notifier.play("")  # TODO: Find sound effects
            #     if enc_flag:
            #         print(time.time() - prev_time, encoder["counts"])
            #         # time.sleep(0.25)

            if is_running() != prev_status:
                if is_running():
                    print("Connection made!")
                    # notifier.play("")  # TODO: Find sound effects
                else:
                    print("Connection lost...")
                    # notifier.play("")  # TODO: Find sound effects
                prev_status = is_running()

            if manual_mode:
                # servo_steering["position"] = int(
                #     50 * (joystick.triggers.L - joystick.triggers.R)) - 23
                servo_steering["position"] = \
                    servo_value([0, 0, 0], [1, -5.34 / 90 * joystick.mainStick.x])
                print(time.time() - prev_time, servo_steering["position"])
            else:
                gps_x, gps_y, change_dist, shifted_yaw = interpreter.convert(
                    gps["lat"], gps["long"], encoder["counts"], imu["yaw"])
                current_time = time.time()
                x, y, vx, vy, ax, ay = kfilter.update(gps_x, gps_y, change_dist,
                        enc_flag, imu["accel_x"], imu["accel_y"], shifted_yaw,
                        current_time - prev_time, imu_flag, gps_flag)
                prev_time = current_time
                goal_x, goal_y = binder.bind((x, y))
                servo_steering["position"] = \
                    servo_value((x, y, shifted_yaw), (goal_x, goal_y))
                if print_data:
                    print(goal_x - x, goal_y - y, shifted_yaw, servo_steering["position"])

            if log_data:
                log.add_data(imu, imu_flag)
                log.add_data(encoder, gps_flag)
                log.add_data(gps, enc_flag)
                log.add_data(servo_steering)
                # log.add_data(servo_brakes)
                log.end_row()

            time.sleep(0.001)

    except KeyboardInterrupt:
        traceback.print_exc()
    finally:
        stop()
        notifier.play("PuzzleDone.wav")
        while notifier.any_playing():
            time.sleep(0.005)


if __name__ == '__main__':
    print(__doc__)
    main(log_data=True, manual_mode=False, print_data=True)
