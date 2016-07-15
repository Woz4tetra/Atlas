import sys
import time

sys.path.insert(0, '../')

use_camera = "F"

if use_camera == "T":
    from vision.camera import Camera
    capture = Camera(480, 320)
else:
    from vision.video import Video
    capture = Video(4, loop_video=True)

paused = False
time_start = time.time()

while True:
    if not paused:
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

capture.stop()
