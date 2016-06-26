import sys

sys.path.insert(0, '../')

from vision.camera import Camera
from vision.video import Video

use_camera = True

if use_camera:
    capture = Camera(480, 320)
else:
    capture = Video(0)
while True:
    capture.get_frame()
    key = capture.key_pressed()
    if key == 'q':
        break
    elif key == 's':
        capture.save_frame()
    elif key == 'v':
        if not capture.recording:
            capture.start_recording()
        else:
            capture.stop_recording()

##    if capture.recording:
##        capture.record_frame()

    capture.show_frame()

capture.stop()
