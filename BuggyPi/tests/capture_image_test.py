
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2

camera = PiCamera()
raw_capture = PiRGBArray(camera)

time.sleep(0.1)

camera.capture(raw_capture, format='bgr')
image = raw_capture.array

cv2.imshow("Image", image)
cv2.waitKey(0)
