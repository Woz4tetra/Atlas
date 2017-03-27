#######################################
# Code coded by Mike Doty
#
# If you want trackball checking, you will
# have to code it yourself.  Sorry!
#
# Oh, and it just grabs the first joystick.
#   Yes, that makes me lazy.
#
# Released February 8, 2008.
#######################################
 
import pygame
from pygame.locals import *
 
class App:
    def __init__(self):
        pygame.init()
 
        pygame.display.set_caption("Joystick Analyzer")
 
        # Set up the joystick
        pygame.joystick.init()
 
        self.my_joystick = None
        self.joystick_names = []
 
        # Enumerate joysticks
        for i in range(0, pygame.joystick.get_count()):
            self.joystick_names.append(pygame.joystick.Joystick(i).get_name())
 
        print(self.joystick_names)
 
        # By default, load the first available joystick.
        if (len(self.joystick_names) > 0):
            self.my_joystick = pygame.joystick.Joystick(0)
            self.my_joystick.init()
 
        max_joy = max(self.my_joystick.get_numaxes(), 
                      self.my_joystick.get_numbuttons(), 
                      self.my_joystick.get_numhats())
 
        self.screen = pygame.display.set_mode( (max_joy * 30 + 10, 170) )
 
        self.font = pygame.font.SysFont("Courier", 20)
        self.small_font = pygame.font.SysFont("Courier", 12)
 
    # A couple of joystick functions...
    def check_axis(self, p_axis):
        if (self.my_joystick):
            if (p_axis and self.my_joystick.get_numaxes()):
                return self.my_joystick.get_axis(p_axis)
 
        return 0
 
    def check_button(self, p_button):
        if (self.my_joystick):
            if (p_button and self.my_joystick.get_numbuttons()):
                return self.my_joystick.get_button(p_button)
 
        return False
 
    def check_hat(self, p_hat):
        if (self.my_joystick):
            if (p_hat and self.my_joystick.get_numhats()):
                return self.my_joystick.get_hat(p_hat)
 
        return (0, 0)
 
    def draw_text(self, text, x, y, color, font=None):
        if font is None:
            font = self.font
        surface = font.render(text, True, color, (0, 0, 0))
        surface.set_colorkey((0, 0, 0))
 
        self.screen.blit(surface, (x, y))
 
    def center_text(self, text, x, y, color):
        surface = self.font.render(text, True, color, (0, 0, 0))
        surface.set_colorkey( (0, 0, 0) )
 
        self.screen.blit(surface, (x - surface.get_width() / 2, 
                                   y - surface.get_height() / 2))
 
    def main(self):
        while (True):
            self.g_keys = pygame.event.get()
 
            self.screen.fill(0)
 
            for event in self.g_keys:
                if (event.type == KEYDOWN and event.key == K_ESCAPE):
                    self.quit()
                    return
 
                elif (event.type == QUIT):
                    self.quit()
                    return
 
            self.draw_text("Joystick Name:  %s" % self.joystick_names[0], 
                           5, 5, (0, 255, 0))
 
            self.draw_text("Axes (%d)" % self.my_joystick.get_numaxes(), 
                           5, 25, (255, 255, 255))

            for i in range(0, self.my_joystick.get_numaxes()):
                if self.my_joystick.get_axis(i):
                    color_channel = int(255 * self.my_joystick.get_axis(i))
                    if color_channel < 0:
                        color = (abs(color_channel), 0, 0)
                    else:
                        color = (0, 0, color_channel)
                    pygame.draw.circle(self.screen, color,
                                       (20 + (i * 50), 60), 10, 0)
                    self.draw_text("%0.2f" % self.my_joystick.get_axis(i),
                                   10 + (i * 50), 65, (255, 255, 255), self.small_font)
                else:
                    pygame.draw.circle(self.screen, (255, 0, 0), 
                                       (20 + (i * 30), 60), 10, 0)
 
                self.center_text("%d" % i, 20 + (i * 50), 60, (255, 255, 255))
 
            self.draw_text("Buttons (%d)" % self.my_joystick.get_numbuttons(), 
                           5, 75, (255, 255, 255))
 
            for i in range(0, self.my_joystick.get_numbuttons()):
                if (self.my_joystick.get_button(i)):
                    pygame.draw.circle(self.screen, (0, 0, 200), 
                                       (20 + (i * 30), 110), 10, 0)
                else:
                    pygame.draw.circle(self.screen, (255, 0, 0), 
                                       (20 + (i * 30), 110), 10, 0)
 
                self.center_text("%d" % i, 20 + (i * 30), 110, (255, 255, 255))
 
            self.draw_text("POV Hats (%d)" % self.my_joystick.get_numhats(), 
                           5, 125, (255, 255, 255))
 
            for i in range(0, self.my_joystick.get_numhats()):
                if (self.my_joystick.get_hat(i) != (0, 0)):
                    pygame.draw.circle(self.screen, (0, 0, 200), 
                                       (20 + (i * 30), 150), 10, 0)
                else:
                    pygame.draw.circle(self.screen, (255, 0, 0), 
                                       (20 + (i * 30), 150), 10, 0)
 
                self.center_text("%d" % i, 20 + (i * 30), 100, (255, 255, 255))
 
            pygame.display.flip()
 
    def quit(self):
        pygame.display.quit()
 
app = App()
app.main()
