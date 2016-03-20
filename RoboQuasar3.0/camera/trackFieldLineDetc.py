
import cv2 
import numpy as np


# original = cv2.imread("/Users/ElimZ/Desktop/buggy/pic/tri2.png")
cap = cv2.VideoCapture("/Users/ElimZ/Desktop/buggy/trackFieldVid1.MOV")
# length = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

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

    # cv2.namedWindow('lines')
    # cv2.createTrackbar('start', 'lines', 0, length, onChange)

    # start = cv2.getTrackbarPos('start', 'lines')
    # cap.set(cv2.CAP_PROP_POS_FRAMES, start)

    while(True):
        ret, original = cap.read()

        # resize
        if original is not None:
            (y, x) = original.shape[0], original.shape[1]
            original = cv2.resize(original, (x/3, y/3))
        # change color
        img = cv2.cvtColor(original, cv2.COLOR_BGR2GRAY)
        img = cv2.medianBlur(img, 5)
        # cv2.imshow('medianBlur', img)
        img = cv2.Canny(img, 1, 100)
        # cv2.imshow('Canny', img)
   
        lines = cv2.HoughLines(img, rho=1, theta=np.pi / 180,
                               threshold=100,
                               min_theta= -70 * np.pi / 180,
                               max_theta= 70 * np.pi / 180)
        # collecting angle (-70, 70) deg

        # drawing lines
        if lines is not None:
            for line in lines:
                rho, theta = line[0][0], line[0][1]
                currentLine = findLineCoord(rho, theta)
                pt1 = currentLine[0]
                pt2 = currentLine[1]
                cv2.line(original, pt1, pt2, (255,0,0),2)
        cv2.imshow('lines', original)

        # # convert img to same dimension;
        # img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        # both = np.concatenate((img, original))
        # cv2.imshow('Lines', both)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

main()


