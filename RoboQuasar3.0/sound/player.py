import sys
import os
import pygame

sys.path.insert(0, '../')

import config

pygame.mixer.init()

class TunePlayer():
    def __init__(self):
        self.tunes = {}
        tunes_dir = config.get_dir("tunes")
        for tune_name in os.listdir(tunes_dir):
            self.tunes[tune_name] = pygame.mixer.Sound(tunes_dir + tune_name)

    def play(self, tune_name, loops=0):
        self.tunes[tune_name].play(loops)

    def stop(self):
        self.tunes[tune_name].stop()
