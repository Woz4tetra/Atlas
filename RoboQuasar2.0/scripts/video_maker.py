"""
    Written by Ben Warwick
    
    video_maker.py, written for the Self-Driving Buggy Project
    Version 11/24/2015
    =========
    
    A little script application that allows for easy OpenCV video recording
    and converting a collection of images to a video.
"""

import argparse
import time
import cv2
import os
import sys
import re

sys.path.insert(0, '../')
from camera import capture
import config


def natural_sort(l):
    convert = lambda text: int(text) if text.isdigit() else text.lower()
    alphanum_key = lambda key: [convert(c) for c in re.split('([0-9]+)', key)]
    return sorted(l, key=alphanum_key)


def write_from_images(input_dir, output_dir, fps, video_name,
                      add_timestamp, format,
                      width, height):
    if input_dir == None:
        input_dir = config.get_dir(":images")
    if output_dir == None:
        output_dir = config.get_dir(":videos")
    if fps == None:
        fps = 5
    if add_timestamp == None:
        add_timestamp = True

    files = natural_sort(os.listdir(input_dir))

    images = []

    print("Using files:")
    for file in files:
        image = cv2.imread(input_dir + "/" + file)
        if image != None:
            print(file)
            if width is None and height is None:
                height, width = image.shape[0:2]
            else:
                image = cv2.resize(image, (width, height))
            images.append(image)

    print("Number of frames:", len(images))
    if len(images) == 0:
        print("No images found!")
        quit()

    if format == None:
        format = 'avi'

    if format != "mov" and format != "m4v":
        codec = "MJPG"
    else:
        codec = "mp4v"

    if video_name == None:
        video_name = ""
    elif video_name != None and add_timestamp == True:
        video_name += " "

    if video_name == None or add_timestamp == True:
        video_name += time.strftime("%c").replace(":", ";") + "." + format

    fourcc = cv2.VideoWriter_fourcc(*codec)
    video = cv2.VideoWriter()
    video.open(output_dir + video_name, fourcc, fps,
               (width, height), True)

    print("Initialized video named '%s'.\nIt will be written to:\n%s" % (
        video_name, output_dir))

    for image in images:
        video.write(image)

    video.release()

    print("Success!!!")

def video_from_camera(output_dir, fps, video_name,
                      add_timestamp, format,
                      width, height, camera_num):
    if fps == None:
        fps = 20
    if add_timestamp == None:
        add_timestamp = True
    if format == None:
        format = 'avi'
    if camera_num == -1:
        camera_num = None

    camera1 = capture.Capture(window_name=video_name,
                              cam_source=camera_num,
                              width=width, height=height)
    
    
    if format != "mov" and format != "m4v":
        codec = "MJPG"
    else:
        codec = "mp4v"
    camera1.initVideoWriter(output_dir=output_dir,
                            fps=fps,
                            video_name=video_name,
                            format=format,
                            codec=codec,
                            includeTimestamp=add_timestamp)
    
    while True:
        frame1 = camera1.getFrame()
        camera1.writeToVideo(frame1)
        camera1.showFrame(frame1)
        key = camera1.getPressedKey()

        if key == 'q' or key == "esc":
            camera1.stopCamera()
            break

if __name__ == '__main__':
    print(__doc__)
    parser = argparse.ArgumentParser()

    parser.add_argument("-s", "--source",
                        help="Create a video from images"
                             "or from a camera feed (supply a camera number)",
                        type=int)

    parser.add_argument("-in", "--input",
                        help="The source of the images to write to video")

    parser.add_argument("-o", "--output",
                        help="Output directory of newly created video")

    parser.add_argument("-n", "--name",
                        help="Name of newly created video")

    parser.add_argument("--fps",
                        help="The speed (frames per second) of the output video",
                        type=int)

    parser.add_argument("-f", "--format",
                        help="The output format of the video")

    parser.add_argument("-wi", "--width",
                        help="The width of the output video",
                        type=int)

    parser.add_argument("-he", "--height",
                        help="The width of the output video",
                        type=int)
    parser.add_argument("-t", "--timestamp",
                        help="Include timestamp in video name",
                        type=bool)

    args = parser.parse_args()

    print((args.input, args.output))

    if args.source == None:
        write_from_images(args.input, args.output,
                          fps=args.fps,
                          video_name=args.name,
                          add_timestamp=args.timestamp,
                          format=args.format,
                          width=args.width,
                          height=args.height)
    else:
        video_from_camera(output_dir=args.output,
                          fps=args.fps,
                          video_name=args.name,
                          add_timestamp=args.timestamp,
                          format=args.format,
                          width=args.width,
                          height=args.height,
                          camera_num=args.source)
    

