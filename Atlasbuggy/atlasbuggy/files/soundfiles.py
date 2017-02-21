import os
import pygame

pygame.mixer.init()


class TunePlayer:
    def __init__(self):
        self.directory = "tunes"
        self.tunes = {}
        for tune_name in os.listdir(self.directory):
            if tune_name.endswith(".wav"):
                self.tunes[tune_name] = pygame.mixer.Sound(os.path.join(self.directory, tune_name))

    def play(self, tune_name):
        self.tunes[tune_name + ".wav"].play()

    def is_playing(self):
        return pygame.mixer.get_busy()

    def stop(self, tune_name):
        self.tunes[tune_name + ".wav"].stop()

    def stop_all(self):
        for tune_name in self.tunes.keys():
            self.tunes[tune_name].stop()
