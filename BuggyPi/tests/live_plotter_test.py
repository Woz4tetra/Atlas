import pygame
import math
import sys

sys.path.insert(0, "../")

from microcontroller.data import *
from microcontroller.dashboard import *
from manual.wiiu_joystick import WiiUJoystick
# from manual.gc_joystick import GCjoystick


def stick_to_servo(x):
    return int(-math.atan2(x, 1) * 180 / (math.pi * 1.85))

width, height = 800, 400
black = (0, 0, 0)
white = (255, 255, 255)

pygame.init()
display = pygame.display.set_mode((width, height))
pygame.display.set_caption("live plotter")
clock = pygame.time.Clock()

exit_flag = False

leds = [Command(command_id, "led " + str(command_id), (0, 2)) for command_id in range(4)]
servo = Command(4, 'servo', (-90, 90))
motors = Command(5, 'motors', (-100, 100))

counts = Sensor(0, 'encoder')
gps = Sensor(1, 'gps', ['lat', 'long', 'found'])
yaw = Sensor(2, 'imu')
altitude = Sensor(3, 'altitude')
checkpoint_num = 0

joystick = WiiUJoystick()

joystick.start()
start(log_data=False, soft_reboot=False)

display.fill(white)

def draw_arrow(x_center, y_center, angle, head_angle=1, length=100):
    pygame.draw.circle(display, white, (int(x_center), int(y_center)), length // 2 + 10)
    x0 = -math.cos(angle) * length / 2 + x_center
    x1 = math.cos(angle) * length / 2 + x_center
    y0 = -math.sin(angle) * length / 2 + y_center
    y1 = math.sin(angle) * length / 2 + y_center

    pygame.draw.line(display, black, (x0, y0), (x1, y1), 10)

    # arrow head:
    x2 = x1 - math.cos(angle + head_angle) * length / 3
    y2 = y1 - math.sin(angle + head_angle) * length / 3
    x3 = x1 - math.cos(angle - head_angle) * length / 3
    y3 = y1 - math.sin(angle - head_angle) * length / 3
    pygame.draw.line(display, black, (x1, y1), (x2, y2), 10)
    pygame.draw.line(display, black, (x1, y1), (x3, y3), 10)


x = width / 2
y = height / 2
angle = 0

try:
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

        draw_arrow(x, y, -yaw.get())

        pygame.display.update()

        clock.tick(60)

        if joystick.get_button('B'):
            print("Aborted by user")
            break

        servo.set(stick_to_servo(joystick.get_axis("left x")))
        value = -joystick.get_axis("left y")
        if value != 0 and abs(value) < 0.8:
            value = 0.8 * ((value > 0) - (value < 0))
        motors.set(int(value * 100))

        print("%0.4f, %5.0i, (%0.6f, %0.6f, %i), motors: %3.0i, servo: %3.0i, %3.6fm    " % (yaw.get(), counts.get(),
              gps.get('lat'), gps.get('long'), gps.get('found'), motors.get(), servo.get(), altitude.get()), end='\r')

except KeyboardInterrupt:
    pass

joystick.stop()
stop()
print("\n------------\n\n")

pygame.quit()
