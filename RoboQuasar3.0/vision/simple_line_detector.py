'''
This file is sufficient to read a video, sets video position using a slider, 
sets color and brightness, and does line detection using houghlines. 

Great starting point for futher analysis on lines and Buggy heading. 

'''



import cv2 
import numpy as np 

cap = cv2.VideoCapture("/Users/ElimZ/Desktop/buggy/trackFieldVid1.MOV")
length = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

def onChange(trackbarVal):
    cap.set(cv2.CAP_PROP_POS_FRAMES, trackbarVal)
    err, frame = cap.read()
    # cv2.imshow('video', frame)


def findLineCoord(rho, theta):
    # turn avgLines into avgLinesCoord =[(x1, y1), (x2, y2)]
    a = np.cos(theta)
    b = np.sin(theta)
    x0 = a * rho
    y0 = b * rho
    x1 = int(x0 + 1000 * -b)
    y1 = int(y0 + 1000 * a)
    x2 = int(x0 - 1000 * -b)
    y2 = int(y0 - 1000 * a)
    return [(x1, y1), (x2, y2)]

def main():

    cv2.namedWindow('lines')
    cv2.createTrackbar('start', 'lines', 0, length, onChange)

    start = cv2.getTrackbarPos('start', 'lines')
    cap.set(cv2.CAP_PROP_POS_FRAMES, start)


    while (True):

        # Set video to trackbar position
        # trackbarVal =int(cv2.getTrackbarPos('position', 'Video')) 
        # if (trackbarVal != prev_TrackbarVal):
        #     cap.set(cv2.CAP_PROP_POS_FRAMES, trackbarVal)
        #     prev_TrackbarVal = trackbarVal

        ret, original = cap.read() 

        # 2. Enhance the brightness, MAGIC NUMBER 60
        hsv = cv2.cvtColor(original, cv2.COLOR_BGR2HSV)
        hsv[:,:,2] += 60
        original = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

        # 3. Resize
        if original is not None:
            (y, x) = original.shape[0], original.shape[1]
            original = cv2.resize(original, (x/2, y/2)) 


         # change color
        img = cv2.cvtColor(original, cv2.COLOR_BGR2GRAY)
        img = cv2.medianBlur(img, 5)
        img = cv2.Canny(img, 1, 100)
   
        lines = cv2.HoughLines(img, rho=1, theta=np.pi / 180,
                               threshold=100,
                               min_theta= -70 * np.pi / 180,
                               max_theta= 70 * np.pi / 180)
        # collecting angle (-70, 70) deg

        # drawing lines
        if lines is not None:
            for line in lines:
                rho, theta = line[0][0], line[0][1]
                # print (rho, theta)
                currentLine = findLineCoord(rho, theta)
                pt1 = currentLine[0]
                pt2 = currentLine[1]
                cv2.line(original, pt1, pt2, (255,0,0),2)
        cv2.imshow('lines', original)



        if cv2.waitKey(1) & 0xFF == ord('q'):
            print "quit key pressed"
            break
    cap.release()
    cv2.destroyAllWindows()

main()




