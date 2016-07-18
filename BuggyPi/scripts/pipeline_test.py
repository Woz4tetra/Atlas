import sys

sys.path.insert(0, '../')

from robots.videobot import VideoBot
from robots.pipeline_config import *
from vision.pipeline import Pipeline

if __name__ == '__main__':
    if len(sys.argv) == 2:
        file_name = sys.argv[1]
        directory = None
    elif len(sys.argv) == 3:
        file_name = sys.argv[1]
        directory = sys.argv[2]
    else:
        file_name = 1
        directory = "rc_car"
        # file_name = ":random"
        # directory = "trackfield"
        # file_name = 0
        # directory = None
    try:
        file_name = int(file_name)
    except ValueError:
        pass

    video_bot = VideoBot(file_name, directory, properties)
    pipeline = Pipeline(video_bot.capture.width, video_bot.capture.height,
                        video_bot.capture.enable_draw)

    while True:
        if not video_bot.update():
            break

        if not video_bot.capture.paused or video_bot.capture.slider_moved():
            if not video_bot.show_original:
                frame = pipeline.update(video_bot.capture)
                video_bot.capture.show_frame(frame)
            else:
                video_bot.capture.show_frame()

