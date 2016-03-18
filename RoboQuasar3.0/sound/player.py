import sys
import os
import pygame

sys.path.insert(0, '../')

import config

pygame.mixer.init()

class TunePlayer():
    def __init__(self):
        self.tunes = {}
        tunes_dir = config.get_dir(":tunes")
        for tune_name in os.listdir(tunes_dir):
            self.tunes[tune_name] = pygame.mixer.Sound(tunes_dir + tune_name)

        pygame.mixer.set_num_channels(8)
        self.channel = pygame.mixer.Channel(5)

    def play(self, tune_name):
        self.channel.play(self.tunes[tune_name])

    def stop(self, tune_name):
        self.channel.stop(self.tunes[tune_name])

    def any_playing(self):
        return self.channel.get_busy()
