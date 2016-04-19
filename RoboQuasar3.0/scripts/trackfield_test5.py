import sys
import traceback

sys.path.insert(0, "../")

from microcontroller.data import *
from microcontroller.dashboard import *

from analyzers.logger import Recorder
from analyzers.converter import Converter
from analyzers.kalman_filter import HeadingFilter, PositionFilter
from analyzers.binder import Binder

from controllers.joystick import joystick_init
from controllers.servo_map import state_to_servo

from sound.player import TunePlayer


def main(log_data=True, manual_mode=True):
    # ----- initialize runner -----

    gps = Sensor(1, ['lat', 'long', 'found'])
    encoder = Sensor(2, 'counts')
    imu = Sensor(3, 'yaw')

    servo_steering = Command(0, 'position', (-90, 90))

    joystick = joystick_init()
    notifier = TunePlayer()

    start(use_handshake=False)

    print("Wait for the GPS to lock on, then press A")

    while not gps['found'] and not joystick.buttons.A:
        time.sleep(0.005)
        joystick.update()
    print(gps['lat'], gps['long'])
    notifier.play("bloop")
    time.sleep(0.05)
    while not joystick.buttons.A:
        time.sleep(0.005)
        joystick.update()

    reset()

    prev_status = is_running()
    prev_gps_status = gps['found']

    prev_time = time.time()

    # ----- initialize converters and filters -----

    converter = Converter(gps['long'], gps['lat'], 0.000003, encoder["counts"])
    heading_filter = HeadingFilter()
    position_filter = PositionFilter()

    binder = Binder("Track Field Map.csv", gps['long'], gps['lat'])
    bind_x, bind_y = 0, 0

    if log_data:
        log = Recorder(directory="Test Day 6")
    else:
        log = None

    notifier.play("ding")

    # ----- main loop -----
    try:
        while True:
            joystick.update()
            if joystick.buttons.B:
                break

            if joystick.buttons.X:
                manual_mode = not manual_mode
                while joystick.buttons.X: pass

            # ----- status notifiers -----
            if encoder.received():
                print(encoder["counts"])
            if prev_gps_status != gps['found']:
                if gps['found']:
                    notifier.play("short victory")
                    print("gps found")
                else:
                    notifier.play("saved")
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
                notifier.play("click")
                gps_heading, bind_heading = converter.convert_heading(
                    gps["long"], gps["lat"], bind_x, bind_y
                )
                gps_x, gps_y, enc_dist = converter.convert_position(
                    gps["long"], gps["lat"], encoder["counts"]
                )

                kalman_heading = heading_filter.update(
                    gps_heading, bind_heading, imu["yaw"]
                )
                kalman_x, kalman_y = position_filter.update(
                    gps_x, gps_y, enc_dist,
                    time.time() - prev_time,
                    kalman_heading
                )
                prev_time = time.time()

                bind_x, bind_y = binder.bind((kalman_x, kalman_y))

            if manual_mode:
                servo_steering["position"] = \
                    state_to_servo([0, 0, 0],
                                   [1, -5.34 / 90 * joystick.mainStick.x])
            else:
                servo_steering["position"] = \
                    state_to_servo([kalman_x, kalman_y, kalman_heading],
                                   [bind_x, bind_y])
            time.sleep(0.005)

            if log_data:
                log["gps long"] = gps["long"]
                log["gps lat"] = gps["lat"]
                log["gps found"] = gps["found"]
                log["gps sleep"] = gps.sleep_time

                log["gps x"] = gps_x
                log["gps y"] = gps_y
                log["gps bearing"] = gps_heading

                log["imu yaw"] = imu["yaw"]
                log["imu sleep"] = imu.sleep_time

                log["encoder"] = encoder["counts"]
                log["encoder sleep"] = encoder.sleep_time
                log["encoder dist"] = enc_dist

                log["bind x"] = bind_x
                log["bind y"] = bind_y
                log["bind heading"] = bind_heading

                log["kalman x"] = kalman_x
                log["kalman y"] = kalman_y
                log["kalman heading"] = kalman_heading

                log["servo"] = servo_steering["position"]

                log.add_row()
    except KeyboardInterrupt:
        traceback.print_exc()
    finally:
        stop()
        joystick.stop()
        if log_data:
            log.close()

if __name__ == '__main__':
    print(__doc__)
    main()
