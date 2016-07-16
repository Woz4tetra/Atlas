import sys
import time

import cv2

sys.path.insert(0, '../')

from vision.video import Video
import project

capture = Video('Thu Jul 14 21;09;54 2016.avi', loop_video=True)

drag_start = None
selection = None
current_image = 1
jump_dist = 1
image_shape = None
paused = True
slide_show = True
frame_changed = False


def on_mouse(event, x, y, flags, param):
    global drag_start, selection
    if event == cv2.EVENT_LBUTTONDOWN:
        drag_start = x, y
        selection = None
    elif event == cv2.EVENT_LBUTTONUP:
        if x == drag_start[0] and y == drag_start[1]:
            selection = None
            update_rect()
        drag_start = None
    elif drag_start:
        if flags & cv2.EVENT_FLAG_LBUTTON:
            min_pos = min(drag_start[0], x), min(drag_start[1], y)
            max_pos = max(drag_start[0], x), max(drag_start[1], y)
            selection = [min_pos[0], min_pos[1], max_pos[0], max_pos[1]]

            update_rect()

        else:
            drag_start = None


def update_rect():
    if selection != None:
        capture.show_frame(cv2.rectangle(capture.frame.copy(),
                                         (selection[0], selection[1]),
                                         (selection[2], selection[3]),
                                         (255, 255, 255), 1))
    else:
        capture.show_frame()


cv2.setMouseCallback(capture.window_name, on_mouse)

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
    # if key != -1:
    #     print(key, repr(key))
    if key == 'q' or key == "esc":
        break
    elif key == ' ':
        if paused:
            print("%i: ...Video unpaused" % capture.frame_num)
        else:
            print("%i: Video paused..." % capture.frame_num)
        paused = not paused
    elif key == 'v':
        if not capture.recording:
            capture.start_recording()
        else:
            capture.stop_recording()
    elif key == '.':
        if jump_dist == 1:
            jump_dist = 10
            print("Long jump for selection rectangle set")
        else:
            jump_dist = 1
            print("Short jump for selection rectangle set")

    if paused:
        if key == 'enter':
            cropped = capture.frame[selection[1]: selection[3],
                      selection[0]: selection[2]]
            if image_shape is not None:
                cropped = cv2.resize(cropped, image_shape)
            capture.save_frame(cropped,
                               directory=project.get_dir(
                                   ":images") + "positives/")
            capture.increment_frame()
            capture.get_frame()
            update_rect()
            print("Positive frame saved!")
        elif key == 'g':
            if selection is not None:
                selection[0] -= jump_dist
                selection[2] += jump_dist
                selection[1] -= jump_dist
                selection[3] += jump_dist

                if selection[0] < 0:
                    selection[0] = 0
                if selection[2] >= capture.frame.shape[1]:
                    selection[2] = capture.frame.shape[1] - 1
                if selection[1] < 0:
                    selection[1] = 0
                if selection[3] >= capture.frame.shape[0]:
                    selection[3] = capture.frame.shape[0] - 1
                update_rect()
        elif key == 's':
            if selection is not None:
                selection[0] += jump_dist
                selection[2] -= jump_dist
                selection[1] += jump_dist
                selection[3] -= jump_dist

                if selection[0] > selection[2]:
                    selection[0] = selection[2] - 1
                if selection[1] > selection[3]:
                    selection[1] = selection[3] - 1
                update_rect()

        elif key == 'down':
            if selection is not None:
                selection[1] += jump_dist
                selection[3] += jump_dist
                update_rect()
        elif key == 'up':
            if selection is not None:
                selection[1] -= jump_dist
                selection[3] -= jump_dist
                update_rect()
        elif key == 'right':
            if selection is None:
                capture.get_frame()
                capture.show_frame()
            else:
                selection[0] += jump_dist
                selection[2] += jump_dist
                update_rect()
        elif key == 'left':
            if selection is None:
                capture.decrement_frame()
                capture.get_frame(False)
                capture.show_frame()
            else:
                selection[0] -= jump_dist
                selection[2] -= jump_dist
                update_rect()

    if capture.recording:
        capture.record_frame()

    if frame_changed:
        update_rect()
        # capture.show_frame()
        frame_changed = False
    else:
        time.sleep(0.005)

capture.stop()
