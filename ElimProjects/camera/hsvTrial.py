# HSVTrial.py in Atlas folder

'''
Written by Elim Zhang

Some code adapted from CV tutorial for Build 18
presented by Esha Uboweja

Self-Driving Buggy Rev. 6 for Self-Driving Buggy Project
Version 2/17/ 2016

=========
Trial for using HSV to select certain colors for edge detection purposes;

Use trackbar to manually select bounds;
Currenly works for video and picture inputs.

'''
# -------------------------------------------------

import cv2
import numpy as np

""" Note: use escape key to quit window.""" 

def nothing(x): pass
cv2.namedWindow('segmented')

# create trackbars for color change
cv2.createTrackbar('H_low','segmented',0, 180, nothing) # hmax = 180
cv2.createTrackbar('S_low','segmented', 0, 255, nothing)
cv2.createTrackbar('V_low','segmented', 0, 255, nothing)

cv2.createTrackbar('H_high','segmented',0, 180, nothing) # hmax = 180
cv2.createTrackbar('S_high','segmented', 0, 255, nothing)
cv2.createTrackbar('V_high','segmented', 0, 255, nothing)

# create switch for ON/OFF functionality
switch = "0 : OFF \n1 : ON"
cv2.createTrackbar(switch, "segmented", 0, 1, nothing)

# cap = cv2.VideoCapture(0) # web cam as video source
cap = cv2.VideoCapture("Videos/Orca 10-10 roll 4.mov")

while(1):
    ret, original = cap.read()
    segmented = np.zeros(original.shape, np.uint8) 

    (y, x) = original.shape[0], original.shape[1]
    if (y >= 360):
        original = cv2.resize(original, (x, y))
    else: 
        original = cv2.resize(original, (x, y))
    hsv = cv2.cvtColor(original, cv2.COLOR_BGR2HSV)
    result = original

    # get current positions of four trackbars
    loH = cv2.getTrackbarPos('H_low','segmented')
    loS = cv2.getTrackbarPos('S_low','segmented')
    loV = cv2.getTrackbarPos('V_low','segmented')
    hiH = cv2.getTrackbarPos('H_high','segmented')
    hiS = cv2.getTrackbarPos('S_high','segmented')
    hiV = cv2.getTrackbarPos('V_high','segmented')
    s = cv2.getTrackbarPos(switch, "Segmented")
    
    low = np.array([loH, loS, loV])
    high = np.array([hiH, hiS, hiV])

    if s: 
        mask = cv2.inRange(hsv, low, high)
        result  = cv2.bitwise_and(original, original, mask = mask)

    cv2.imshow('segmented', result)

    key = cv2.waitKey(20) & 0xFF
    if key == 27 or chr(key) == 'q':
        break
        cv2.destroyAllWindows()
        cap.release()

cap.release()
cv2.destroyAllWindows()




