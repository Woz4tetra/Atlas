import sys
import time

sys.path.insert(0, '../')

from vision.pipeline import MeanShiftTracker

use_camera = "F"

if use_camera == "T":
    from vision.camera import Camera
    capture = Camera(480, 320)
else:
    from vision.video import Video
    capture = Video('Thu Jul 14 21;09;54 2016.avi', loop_video=True)

paused = True
slide_show = True
frame_changed = False

tracker = MeanShiftTracker(capture.get_frame(False))

while True:
    if not paused:
        if capture.get_frame() is None:
            break
        frame_changed = True
    elif capture.frame_num != capture.slider_num:
        capture.get_frame(False)
        frame_changed = True
    
    if slide_show:
        paused = True

    key = capture.key_pressed()

    if key == 'q' or key == "esc":
        break
    elif key == ' ':
        if paused:
            print("%i: ...Video unpaused" % (capture.frame_num))
        else:
            print("%i: Video paused..." % (capture.frame_num))
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
    
    if frame_changed:
        frame = tracker.update(capture.frame)
    
        capture.show_frame(frame)
        frame_changed = False
    else:
        time.sleep(0.005)

capture.stop()

