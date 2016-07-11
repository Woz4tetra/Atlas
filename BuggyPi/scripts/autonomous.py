import os
import sys

sys.path.insert(0, "../")

from microcontroller.data import *
from microcontroller.comm import *
from manual.wiiu_joystick import WiiUJoystick
from vision.camera import Camera
from microcontroller.logger import get_map
from navigation.buggypi_filter import BuggyPiFilter


def log_folder():
    month = time.strftime("%b")
    day = time.strftime("%d")
    year = time.strftime("%Y")
    return "%s %s %s" % (month, day, year)


def stick_to_servo(x):
    return int(-math.atan2(x, 1) * 180 / (math.pi * 1.85))


def button_dn(button, params):
    global manual_mode, checkpoint_num, gps, communicator
    if button == 'B':
        manual_mode = not manual_mode
        print("Switching to", "manual mode!" if manual_mode else "autonomous!")
    if button == 'A':
        communicator.record('checkpoint', num=checkpoint_num,
                            long=gps.get("long"), lat=gps.get("lat"))
        checkpoint_num += 1
        print(
            "--------\nCheckpoint reached! %s\n--------" % str(checkpoint_num))


def joy_changed(axis, value, params):
    global manual_mode, servo, motors, joystick
    if manual_mode:
        if axis == "left x":
            servo.set(stick_to_servo(value))
        if axis == "left y":
            value = -joystick.get_axis("left y")
            if value != 0:
                value = 1 * ((value > 0) - (value < 0))
            motors.set(int(value * 100))


encoder = Sensor(0, 'encoder', 'counts')
gps = Sensor(1, 'gps', ['lat', 'long', 'altitude', 'found'])
yaw = Sensor(2, 'imu', 'yaw')
altitude = Sensor(3, 'altitude', 'altitude')
checkpoint_num = 0

sensor_pool = SensorPool(encoder, gps, yaw, altitude)
log_data = True
if len(sys.argv) == 2 and sys.argv[1] == 'no-log':
    log_data = False
communicator = Communicator(sensor_pool, address='/dev/ttyAMA0',
                            log_data=log_data, log_dir=log_folder())

if not communicator.initialized:
    raise Exception("Communicator not initialized...")

leds = [Command(command_id, "led " + str(command_id), (0, 2), communicator)
        for command_id in range(4)]
servo = Command(4, 'servo', (-90, 90), communicator)
motors = Command(5, 'motors', (-100, 100), communicator)

joystick = WiiUJoystick(button_down_fn=button_dn,
                        axis_active_fn=joy_changed)
manual_mode = True

enable_draw = True
capture = Camera(320, 240, enable_draw=enable_draw, framerate=32)

checkpoints = get_map()
initial_long, initial_lat = checkpoints[9]

pi_filter = BuggyPiFilter(
    initial_long, initial_lat, 0.0, 6, 0.097, 0.234, 0.88
)


def main():
    if not enable_draw:
        print("Display will now turn off")
        time.sleep(2)
        os.system("sudo echo 1 > /sys/class/backlight/rpi_backlight/bl_power")

    joystick.start()
    communicator.start()

    time_start = time.time()

    paused = False

    try:
        while True:
            if yaw.received():
                print(yaw)
                pi_filter.update_imu(time.time() - time_start, -yaw.get('yaw'))
            if gps.received():
                print(gps)
                pi_filter.update_gps(time.time() - time_start,
                                     gps.get("long"), gps.get("lat"))
            if encoder.received():
                print(encoder)
                pi_filter.update_encoder(time.time() - time_start,
                                         encoder.get("counts"))
            # if altitude.received():
            #     print(altitude)
            # time.sleep(0.05)
            if capture.get_frame() is None:
                break
            key = capture.key_pressed()

            if key == 'q' or key == "esc":
                break
            elif key == ' ':
                if paused:
                    print("%0.4fs: ...Video unpaused" % (
                        time.time() - time_start))
                else:
                    print("%0.4fs: Video paused..." % (
                        time.time() - time_start))
                paused = not paused
            elif key == 's':
                capture.save_frame()
            elif key == 'v':
                if not capture.recording:
                    capture.start_recording()
                else:
                    capture.stop_recording()

            if capture.recording:
                capture.record_frame()

            capture.show_frame()

    except:
        traceback.print_exc()
    finally:
        joystick.stop()
        communicator.stop()
        capture.stop()

        if not enable_draw:
            os.system(
                "sudo echo 0 > /sys/class/backlight/rpi_backlight/bl_power")


if __name__ == '__main__':
    main()
