"""
    Written by Ben Warwick
    
    scripts, written for RoboQuasar1.0
    Version 1/12/2015
    =========
    
    Sample maker script for a classifier. Generates positive and negative
    sample images (with user input). When the 'n' key is pressed, every frame
    displayed on screen will be saved to Images/negatives/ (the same image will
    not be saved twice). When paused (pressed space), highlight the subject to
    be recognized by the classifier using the mouse. To save the image, press
    enter.
    
    Usage
    -----
    python __main__.py
    - or - (in folder directory):
    python selectionf-Driving\ Buggy\ Rev.\ 6

    Keys
    ----
        q, ESC - exit
        SPACE - play/pause video. When unpaused, the program is in write
            negatives mode. If write_negatives is True (toggled by 'n' key),
            the current frame will written to the negatives image directory

            when paused, the program is in write positives mode. Select the
            region of interest using the mouse

        Only works while unpaused:
            n - toggle write_negatives

        Only works while paused:
            arrow keys - shift region of interest by jump distance (shifts
                between 1 and 10 pixels depending on whether long jump is True
                (toggled by the '.' key))
            g - grow the region of interest by jump distance on all sides
            s - shrink the region of interest by jump distance on all sides
            enter - save region of interest to positive images


    unpaused - write background image move to next frame
    hit space or click - pause
    drag - draw rectangle
    hit space - go to unpaused
    hit enter - write image in rectangle, move to next frame

"""

import sys
import cv2

sys.path.insert(0, '../')

import config

from camera import capture

drag_start = None
selection = None
camera1 = capture.Capture(window_name="camera",
                          cam_source="Icarus 10-18 roll 4.mov",
                          # width=720, height=450,
                          # width=427, height=240,
                          frame_skip=10,
                          loop_video=False,
                          # start_frame=31000,
                          quit_on_end=True
                          )
current_image = 1

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
        camera1.showFrame(cv2.rectangle(camera1.frame.copy(),
                                        (selection[0], selection[1]),
                                        (selection[2], selection[3]),
                                        (255, 255, 255), 1))
    else:
        camera1.showFrame()

def run():
    paused = True
    jump_dist = 1
    write_negatives = True
    image_shape = None
    global current_image

    print("write_negatives is " + str(write_negatives))

    if paused == True:
        camera1.getFrame(readNextFrame=False)
        camera1.showFrame()

    cv2.setMouseCallback(camera1.windowName, on_mouse)
    while camera1.isRunning:
        if paused == False:
            camera1.getFrame()
            camera1.showFrame()
            if write_negatives == True:
                if image_shape is not None:
                    frame = cv2.resize(camera1.frame, image_shape)
                else:
                    frame = camera1.frame
                camera1.saveFrame(frame,
                                  directory=config.get_dir(
                                      ":images") + "negatives/")


        key = camera1.getPressedKey()
        if key == 'q' or key == "esc":
            camera1.stopCamera()
            break
        elif key == ' ':
            if paused:
                camera1.frameSkip = 15
                print("...Video unpaused")
            else:
                camera1.frameSkip = 0
                print("Video paused...")
                update_rect()

            paused = not paused
        elif key == '.':
            if jump_dist == 1:
                jump_dist = 10
                print("Long jump for selection rectangle set")
            else:
                jump_dist = 1
                print("Short jump for selection rectangle set")
        if key == "1":
            current_image = 1
            print("current image is", current_image)
        elif key == "2":
            current_image = 2
            print("current image is", current_image)
        elif key == "3":
            current_image = 3
            print("current image is", current_image)
        elif key == "4":
            current_image = 4
            print("current image is", current_image)

        if paused:
            if key == 'enter':
                cropped = camera1.frame[selection[1]: selection[3],
                                        selection[0]: selection[2]]
                if image_shape is not None:
                    cropped = cv2.resize(cropped, image_shape)
                print("current image is", current_image)
                if current_image == 1:
                    print("1")
                    camera1.saveFrame(cropped,
                                      directory=config.get_dir(
                                          ":images") + "positives1/")
                elif current_image == 2:
                    print("2")
                    camera1.saveFrame(cropped,
                                      directory=config.get_dir(
                                              ":images") + "positives2/")
                elif current_image == 3:
                    print("3")
                    camera1.saveFrame(cropped,
                                      directory=config.get_dir(
                                              ":images") + "positives3/")
                elif current_image == 4:
                    print("4 ")
                    camera1.saveFrame(cropped,
                                      directory=config.get_dir(
                                              ":images") + "positives4/")
                #camera1.saveFrame(cropped,
                #        directory=config.get_dir(":images") + "positives/")
                camera1.incrementFrame()

                camera1.getFrame()
                update_rect()
                print("Positive frame saved!")

            elif key == 'n':
                write_negatives = not write_negatives
                print("write_negatives is " + str(write_negatives))
            elif key == 'g':
                selection[0] -= jump_dist
                selection[2] += jump_dist
                selection[1] -= jump_dist
                selection[3] += jump_dist
                if selection[0] < 0:
                    selection[0] = 0
                if selection[2] >= camera1.frame.shape[1]:
                    selection[2] = camera1.frame.shape[1] - 1
                if selection[1] < 0:
                    selection[1] = 0
                if selection[3] >= camera1.frame.shape[0]:
                    selection[3] = camera1.frame.shape[0] - 1
                update_rect()

            elif key == 's':
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
                selection[1] += jump_dist
                selection[3] += jump_dist
                update_rect()
            elif key == 'up':
                selection[1] -= jump_dist
                selection[3] -= jump_dist
                update_rect()
            elif key == 'right':
                if selection == None:
                    camera1.getFrame()
                    camera1.showFrame()
                else:
                    selection[0] += jump_dist
                    selection[2] += jump_dist
                    update_rect()
            elif key == 'left':
                if selection == None:
                    camera1.decrementFrame()
                    camera1.getFrame(readNextFrame=False)
                    camera1.showFrame()
                else:
                    selection[0] -= jump_dist
                    selection[2] -= jump_dist
                    update_rect()



if __name__ == '__main__':
    print(__doc__)

    arguments = sys.argv
    if "help" in arguments:
        raise NotImplementedError
    else:
        run()
