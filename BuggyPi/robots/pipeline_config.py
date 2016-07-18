from robots.standard_config import *


def update_camera(robot):
    if robot.enable_camera:
        key = robot.capture.key_pressed()

        if key == 'q' or key == "esc":
            print("quitting...")
            return False
        elif key == ' ':
            if robot.capture.paused:
                print("%0.4fs: ...Video unpaused" % (
                    time.time() - robot.time_start))
            else:
                print("%0.4fs: Video paused..." % (
                    time.time() - robot.time_start))
            robot.capture.paused = not robot.capture.paused
        elif key == 's':
            robot.capture.save_frame()
        elif key == 'v':
            if not robot.capture.recording:
                robot.capture.start_recording()
            else:
                robot.capture.stop_recording()
        elif key == 'o':
            robot.show_original = not robot.show_original
        elif key == 'right':
            robot.capture.increment_frame()
        elif key == 'left':
            robot.capture.decrement_frame()

    return True  # True == don't exit program


properties = update_properties(
    # ----- camera -----
    enable_draw=True,
    cam_width=None,
    cam_height=None,
    use_checkpoints=False,
    use_filter=False,
    update_camera_fn=update_camera,
    show_original=False,
    loop_video=True
)