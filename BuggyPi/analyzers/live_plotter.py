import pygame
import math

width, height = 800, 640
black = (0, 0, 0)
white = (255, 255, 255)

pygame.init()
display = pygame.display.set_mode((width, height))
pygame.display.set_caption("live plotter")
clock = pygame.time.Clock()

exit_flag = False


def draw_arrow(x_center, y_center, angle, length=100):
    x0 = -math.cos(math.radians(angle)) * length / 2 + x_center
    x1 = math.cos(math.radians(angle)) * length / 2 + x_center
    y0 = -math.sin(math.radians(angle)) * length / 2 + y_center
    y1 = math.sin(math.radians(angle)) * length / 2 + y_center

    pygame.draw.line(display, black, (x0, y0), (x1, y1))


x = width / 2
y = height / 2
angle = 0

while not exit_flag:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            exit_flag = True
            break
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_LEFT:
                x -= 5
            elif event.key == pygame.K_RIGHT:
                x += 5
            elif event.key == pygame.K_UP:
                y -= 5
            elif event.key == pygame.K_DOWN:
                y += 5
            elif event.key == pygame.K_0:
                angle -= 5
            elif event.key == pygame.K_1:
                angle += 5

    display.fill(white)

    draw_arrow(x, y, angle)

    pygame.display.update()

    clock.tick(60)

pygame.quit()
