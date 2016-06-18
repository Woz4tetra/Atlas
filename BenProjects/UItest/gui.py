import pygame
# import math
from buffer import SerialBuffer
import traceback

width, height = 800, 640
black = (0, 0, 0)
white = (255, 255, 255)

pygame.init()
display = pygame.display.set_mode((width, height))
pygame.display.set_caption("test gui")
clock = pygame.time.Clock()

exit_flag = False

buffer = SerialBuffer()

text = ""
print(buffer.begin())

# def draw_arrow(x_center, y_center, angle, length=100):
#     x0 = -math.cos(math.radians(angle)) * length / 2 + x_center
#     x1 = math.cos(math.radians(angle)) * length / 2 + x_center
#     y0 = -math.sin(math.radians(angle)) * length / 2 + y_center
#     y1 = math.sin(math.radians(angle)) * length / 2 + y_center
#
#     pygame.draw.line(display, black, (x0, y0), (x1, y1))
#
#
# x = width / 2
# y = height / 2
# angle = 0

font = pygame.font.SysFont('Arial', 10)


class TextRectException:
    def __init__(self, message=None):
        self.message = message

    def __str__(self):
        return self.message

def render_textrect(string, font, rect, text_color, background_color,
                    justification=0):
    """Returns a surface containing the passed text string, reformatted
    to fit within the given rect, word-wrapping as necessary. The text
    will be anti-aliased.

    Takes the following arguments:

    string - the text you wish to render. \n begins a new line.
    font - a Font object
    rect - a rectstyle giving the size of the surface requested.
    text_color - a three-byte tuple of the rgb value of the
                 text color. ex (0, 0, 0) = BLACK
    background_color - a three-byte tuple of the rgb value of the surface.
    justification - 0 (default) left-justified
                    1 horizontally centered
                    2 right-justified

    Returns the following values:

    Success - a surface object with the text rendered onto it.
    Failure - raises a TextRectException if the text won't fit onto the surface.
    """

    final_lines = []

    requested_lines = string.splitlines()

    # Create a series of lines that will fit on the provided
    # rectangle.

    for requested_line in requested_lines:
        if font.size(requested_line)[0] > rect.width:
            words = requested_line.split(' ')
            # if any of our words are too long to fit, return.
            for word in words:
                if font.size(word)[0] >= rect.width:
                    raise TextRectException("The word " + word + " is too long to fit in the rect passed.")
            # Start a new line
            accumulated_line = ""
            for word in words:
                test_line = accumulated_line + word + " "
                # Build the line while the words fit.
                if font.size(test_line)[0] < rect.width:
                    accumulated_line = test_line
                else:
                    final_lines.append(accumulated_line)
                    accumulated_line = word + " "
            final_lines.append(accumulated_line)
        else:
            final_lines.append(requested_line)

            # Let's try to write the text out on the surface.

    surface = pygame.Surface(rect.size)
    surface.fill(background_color)

    accumulated_height = 0
    for line in final_lines:
        if accumulated_height + font.size(line)[1] >= rect.height:
            raise TextRectException("Once word-wrapped, the text string was too tall to fit in the rect.")
        if line != "":
            tempsurface = font.render(line, 1, text_color)
            if justification == 0:
                surface.blit(tempsurface, (0, accumulated_height))
            elif justification == 1:
                surface.blit(tempsurface, (
                (rect.width - tempsurface.get_width()) / 2, accumulated_height))
            elif justification == 2:
                surface.blit(tempsurface, (
                rect.width - tempsurface.get_width(), accumulated_height))
            else:
                raise TextRectException("Invalid justification argument: " + str(
                    justification))
        accumulated_height += font.size(line)[1]

    return surface


text_rect = pygame.Rect((0, 0, width, height))

try:
    while not exit_flag:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                exit_flag = True
                break
            # if event.type == pygame.KEYDOWN:
            #     if event.key == pygame.K_LEFT:
            #         x -= 5
            #     elif event.key == pygame.K_RIGHT:
            #         x += 5
            #     elif event.key == pygame.K_UP:
            #         y -= 5
            #     elif event.key == pygame.K_DOWN:
            #         y += 5
            #     elif event.key == pygame.K_0:
            #         angle -= 5
            #     elif event.key == pygame.K_1:
            #         angle += 5

        # display.fill(white)

        # draw_arrow(x, y, angle)
        print(buffer.get())
        display.blit(render_textrect(text, font, text_rect, white, black), text_rect.topleft)


        pygame.display.update()

        clock.tick(60)
except:
    traceback.print_exc()

buffer.stop()
pygame.quit()
