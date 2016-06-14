
import sys
import math
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2

sys.path.insert(0, "../")

from microcontroller.data import *
from microcontroller.dashboard import *
from manual.wiiu_joystick import WiiUJoystick


def stick_to_servo(x):
    return int(-math.atan2(x, 1) * 180 / math.pi) // 3


def main():
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=(640, 480))

    time.sleep(0.1)

    leds = [Command(command_id, (0, 2)) for command_id in range(4)]
    servo = Command(4, (-90, 90))
    motors = Command(5, (-100, 100))
    
    imu = Sensor(2, 'yaw')

    joystick = WiiUJoystick()

    joystick.start()
    start()

    try:
        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            if joystick.get_button('B'):
                print("Aborted by user")
                break

            servo.set(stick_to_servo(joystick.get_axis("left x")))
            motors.set(int(-joystick.get_axis("right y") * 100))
            
            if joystick.dpad[1] == 1:
                leds[0].set(2)
            if joystick.dpad[1] == -1:
                leds[1].set(2)
            if joystick.dpad[0] == 1:
                leds[2].set(2)
            if joystick.dpad[0] == -1:
                leds[3].set(2)

            print(imu.get('yaw'))

            # grab the raw NumPy array representing the image, then initialize the timestamp
            # and occupied/unoccupied text
            image = frame.array

            # show the frame
            cv2.imshow("Frame", image)
            key = cv2.waitKey(1) & 0xFF

            # clear the stream in preparation for the next frame
            rawCapture.truncate(0)

            # if the `q` key was pressed, break from the loop
            if key == ord("q"):
                print("Aborted by user")
                break


    except KeyboardInterrupt:
        pass
    finally:
        stop()


if __name__ == '__main__':
    main()
