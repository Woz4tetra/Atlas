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
            if tune_name.endswith(".wav"):
                self.tunes[tune_name] = pygame.mixer.Sound(tunes_dir + tune_name)


    def play(self, tune_name):
        self.tunes[tune_name + ".wav"].play()

    def stop(self, tune_name):
        self.tunes[tune_name + ".wav"].stop()

    def stop_all(self):
        for tune_name in self.tunes.keys():
            self.tunes[tune_name].stop()
