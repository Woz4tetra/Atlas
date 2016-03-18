    # applies houghlines or cascade classfier filters on an input camera frame

import numpy as np
import cv2

class RoadWarper:
    def __init__(self, window_name, width, height, warp_points=None):
        if warp_points is None:
            self.pts1 = []
        else:
            self.pts1 = warp_points
        self.pts2 = [[0, 0], [width, 0], [0, height], [width, height]]
        self.M = None
        self.width, self.height = width, height
        cv2.setMouseCallback(window_name, self.on_mouse, self)
        if len(self.pts1) == 4:
            pts1 = np.array(self.pts1, np.float32)
            pts2 = np.array(self.pts2, np.float32)
            self.M = cv2.getPerspectiveTransform(pts1, pts2)

    @staticmethod
    def on_mouse(event, x, y, flags, self):
        if event == cv2.EVENT_LBUTTONDOWN:
            print("Point #%i: %s" % (len(self.pts1), str((x, y))))
            if len(self.pts1) < 3:
                self.pts1.append([x, y])
            elif len(self.pts1) < 4:
                self.pts1.append([x, y])
                print("Points:", self.pts1)
                pts1 = np.array(self.pts1, np.float32)
                pts2 = np.array(self.pts2, np.float32)
                self.M = cv2.getPerspectiveTransform(pts1, pts2)
                print("Warp matrix:", self.M)


    def update(self, frame):
        if 0 < len(self.pts1) < 4:
            for coord in self.pts1:
                frame = cv2.circle(frame, tuple(coord), 2, (255, 0, 0), 2)
        elif self.M is not None:
            frame = cv2.warpPerspective(frame, self.M, (self.width, self.height))
        return frame

    def reset(self):
        print("Points reset")
        self.pts1 = []
        self.M = None



class LineFollower:
    def __init__(self, expected, y_bottom, width, height):
        self.centerRho, self.centerTheta = expected
        self.width, self.height = width, height
        self.yBottom = y_bottom

        self.prevLeftRho, self.prevLeftTheta = None, None
        self.prevRightRho, self.prevRightTheta = None, None

    def isEqual(self, currentRho, currentTheta, 
                      existedRho, existedTheta, tolerance): 
        '''
        if rho and theta are close enough,
        set these lines as equivalent
        To minimize # of lines on screen
        '''
        if (currentTheta - existedTheta) <= tolerance and \
            (currentRho - existedRho)<= tolerance:
            return True
        return False 


    def merge(self, line_set):
        occurance = len(line_set)
        # if occurs more than once, 
        # merge and return a single median (rho, theta)
        if occurance > 1:
            medianTheta = np.median(line_set[0][0])
            medianRho = np.median(line_set[0][1])
            line_set = [occurance, medianRho, medianTheta]
        else: 
            line_set = [occurance, line_set[0][0], line_set[0][1]]
        return line_set

    def findAverageLines(self, lines):
        '''
        findAvgLines is not supposed to draw; 
        use update to blur and find lines, 
        then use findAvglines func to return avgline
        '''
        rightRho, rightTheta, leftRho, leftTheta = [], [], [], []

        # Divide lines into left and right groups, accoridng to sign of gradient 
        for currentLine in lines:
            # print "lines = ", lines
            # print "currentLines = ", currentLine
            # notes on indexing: currentline has format[[x1, y1]]
            (rho, theta)  = (currentLine[0][0], currentLine[0][1])
            if theta > 0: 
                # lines with negative gradient; (y increases downwards in frame)
                leftTheta.append(theta)
                leftRho.append(rho)

            elif theta <= 0:
                rightTheta.append(theta)
                rightRho.append(rho)

        if len(leftRho) != 0:
            avgLeftRho = np.median([leftRho])
            avgLeftTheta = np.median([leftTheta])
        else:
            (avgLeftRho, avgLeftTheta) = (0, 0)

        if len(rightRho) != 0:
            avgRightRho = np.median([rightRho])
            avgRightTheta = np.median([rightTheta])
        else: (avgRightRho, avgRightTheta) = (0, 0)

        # self.avgCenterRho = (avgLeftRho + avgRightRho) / 2.0
        # self.avgCenterTheta = (avgLeftTheta + avgRightTheta) / 2.0

        # Avoid sudden change in values
        # Left: 
        # if self.prevLeftRho != None and self.prevLeftTheta != None:
        #     #  is the second condition redundant?
        #     if avgLeftRho == 0 and avgLeftTheta == 0:
        #         avgLeftRho = self.prevLeftRho
        #         avgLeftTheta = self.prevLeftTheta
        #     elif self.isEqual(avgLeftRho, avgLeftTheta, 
        #                       self.prevLeftRho, self.prevLeftTheta, 100):
        #         self.prevLeftRho = avgLeftRho
        #         self.prevLeftTheta = avgLeftTheta
        #     else: 
        #         avgLeftRho = self.prevLeftRho
        #         avgLeftTheta = self.prevLeftTheta
        # else:
        #     self.prevLeftRho = avgLeftRho
        #     self.prevLeftTheta = avgLeftTheta 

        # # Right
        # if self.prevRightRho != None and self.prevRightTheta != None:
        #     #  is the second condition redundant?
        #     if avgRightRho == 0 and avgRightTheta == 0:
        #         avgRightRho = self.prevRightRho
        #         avgRightTheta = self.prevRightTheta
        #     elif self.isEqual(avgRightRho, avgRightTheta, 
        #                       self.prevRightRho, self.prevRightTheta, 100):
        #         self.prevRightRho = avgRightRho
        #         self.prevRightTheta = avgRightTheta
        #     else: 
        #         avgRightRho = self.prevRightRho
        #         avgRightTheta = self.prevRightTheta
        # else:
        #     self.prevRightRho = avgRightRho
        #     self.prevRightTheta = avgRightTheta 

                


        return [(avgLeftRho, avgLeftTheta), (avgRightRho, avgRightTheta)]

    def findLineCoord(self, rho, theta):
        # turn avgLines into avgLinesCoord =[(x1, y1), (x2, y2)]
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a * rho
        y0 = b * rho
        x1 = int(x0 + 1000 * -b)
        y1 = int(y0 + 1000 * a)
        x2 = int(x0 - 1000 * -b)
        y2 = int(y0 - 1000 * a)
        return (x1, y1, x2, y2)

    def difference(self, expected, actual, y_bottom):
        return 0, 0  # distance difference, theta difference
        ''' need to filter out unneeded lines before taking avg'''
    
    def update(self, frame, draw_avg=True, draw_all=True, 
        maxNumLines = 50, tolerance = 0.01):
        frame = frame[90:360,::]
        frame_lines = cv2.medianBlur(frame, 5)
        frame_lines = cv2.Canny(frame, 1, 150)

        lines = cv2.HoughLines(frame_lines, rho=1, theta=np.pi / 180,
                               threshold=100,
                               min_theta=-70 * np.pi / 180,
                               max_theta=70 * np.pi / 180)
        
        ''' add a merge func here?? ''' 
            

        if lines is not None:
            if draw_all == True:
                '''
                draw lines based on their occurance times
                '''
                for line in lines[:maxNumLines]:
                    rho, theta = line[0][0], line[0][1]

                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a * rho
                    y0 = b * rho
                    x1 = int(x0 + 1000 * -b)
                    y1 = int(y0 + 1000 * a)
                    x2 = int(x0 - 1000 * -b)
                    y2 = int(y0 - 1000 * a)
                    
                    cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                

        # if lines is not None:

            averaged_line = self.findAverageLines(lines[:maxNumLines]) 
            (rho1, theta1) = (averaged_line)[0] 
            (rho2, theta2) = (averaged_line)[1]
            (x1, y1, x2, y2) = self.findLineCoord(rho1, theta1)
            (x3, y3, x4, y4) = self.findLineCoord(rho2, theta2)

            # get coordinates of lines before drawing
            if draw_avg:
                cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.line(frame, (x3, y3), (x4, y4), (0, 255, 0), 2)

        else:
            averaged_line = None, None
        return frame, self.difference((self.centerRho, self.centerTheta
                                       ), averaged_line, self.yBottom)
