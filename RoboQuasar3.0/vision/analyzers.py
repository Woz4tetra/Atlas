# applies houghlines or cascade classfier filters on an input camera frame

import bisect

import cv2
import numpy as np


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
            frame = cv2.warpPerspective(frame, self.M,
                                        (self.width, self.height))
        return frame

    def reset(self):
        print("Points reset")
        self.pts1 = []
        self.M = None


def detect_lines(frame, night_mode, max_lines=None):
    detected = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # detected = cv2.medianBlur(detected, 5)
    # detected = cv2.GaussianBlur(detected, (3, 3), 0)

    # detected = cv2.Sobel(detected, cv2.CV_64F, 1, 0, ksize=3)
    # detected = np.absolute(detected)
    # detected = np.uint8(detected)

    if night_mode:
        detected = cv2.Canny(detected, 1, 100)
        lines = cv2.HoughLines(detected, rho=1.2, theta=np.pi / 180,
                               threshold=100,
                               min_theta=-70 * np.pi / 180,
                               max_theta=70 * np.pi / 180)
    else:
        detected = cv2.GaussianBlur(detected, (7, 7), 0)
        detected = cv2.Canny(detected, 1, 100)
        lines = cv2.HoughLines(detected, rho=1, theta=np.pi / 180,
                               threshold=100,
                               min_theta=-70 * np.pi / 180,
                               max_theta=70 * np.pi / 180)

    if max_lines is not None and lines is not None:
        lines = lines[0:max_lines]
    return lines, cv2.cvtColor(np.uint8(detected), cv2.COLOR_GRAY2BGR)


def threshold_dark_image(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    gray = cv2.equalizeHist(gray)
    gray = cv2.GaussianBlur(gray, (11, 11), 0)

    thresh_val, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_OTSU)
    return binary


def get_contours(binary, epsilon=None):
    """
    Find the contours of an image and return an array of them sorted by increasing perimeter size.

    :param binary: The binary image that contours are to be calculated
    :param epsilon: If specified the approxPolyDP algorithm will be applied to the result.
                    Recommended value is 0.001
    :return: A 2D numpy array of the contours
    """
    binary, contours, hierarchy = cv2.findContours(binary.copy(), cv2.RETR_TREE,
                                                   cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    sig_contours = []
    perimeters = []

    for contour in contours:
        perimeter = cv2.arcLength(contour, closed=False)
        index = bisect.bisect(perimeters, perimeter)
        if epsilon is not None:
            approx = cv2.approxPolyDP(contour, epsilon * perimeter, False)
            sig_contours.insert(index, approx)
        else:
            sig_contours.insert(index, contour)
        perimeters.insert(index, perimeter)

    return sig_contours


def draw_contours(frame, contours):
    return cv2.drawContours(frame.copy(), contours, -1, (255, 255, 255), 3)


def get_pid_goal(lines, offset_x, reference_height, frame=None):
    """
    Returns a goal x and angle according to the specifications of cv_pid.py
    (goal y assumed to be the top of the screen)

    :param lines: an array of rho, theta pairs returned from cv2.HoughLines
    :param offset_x: Center line location
    :param reference_height: The point to reference distance from
    :param frame: frame to draw result on
    :return: goal x in pixels that the buggy should point to, incident angle to
        line (radians). 0 radians is defined down the screen along the center
        line. A negative angle indicates the line is sloping downward
        (positive angle: /, negative angle: \)
    """
    if lines is not None:
        shortest_dist = None
        closest_line = None
        # intersect_angle = 0  # measured from reference line
        draw_pts = None
        for line in lines:
            rho, theta = line[0][0], line[0][1]
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a * rho
            y0 = b * rho
            x1 = int(x0 + 1000 * -b)
            y1 = int(y0 + 1000 * a)

            if x1 - x0 != 0:
                slope_hough = (y1 - y0) / (x1 - x0)
                offset_hough = y0 - (y1 - y0) / (x1 - x0) * x0

            if x1 - x0 != 0 and abs(slope_hough) < 1000:
                intersect_y = slope_hough * offset_x + offset_hough
                distance = reference_height - intersect_y

                # y2 = reference_height
                # x2 = (y2 - offset_hough) / slope_hough

                # angle = np.arctan2(x2 - offset_x, y2 - intersect_y)

                if shortest_dist is None or distance < shortest_dist:
                    closest_line = (slope_hough, offset_hough)
                    shortest_dist = distance
                    # intersect_angle = angle
                    # if frame is not None:
                    #     draw_pts = ((int(offset_x), int(reference_height)),
                    #                 (int(offset_x), int(intersect_y)))
            else:
                distance = reference_height
                if shortest_dist is None or distance < shortest_dist:
                    closest_line = None
                    shortest_dist = distance
                    # intersect_angle = 0.0
                    # if frame is not None:
                    #     draw_pts = ((int(offset_x), 0),
                    #                 (int(offset_x), int(reference_height)))
        if frame is not None:
            # if draw_pts is not None:
            #     cv2.line(frame, draw_pts[0], draw_pts[1], (0, 0, 255), 2)
            if closest_line is not None:
                cv2.circle(frame, (int(-closest_line[1] / closest_line[0]), 0),
                           20, (50, 205, 0), 5)
        cv2.circle(frame, (int(-closest_line[1] / closest_line[0]), 0), 10,
                   (50, 205, 0), 3)

        if closest_line is not None:
            return -closest_line[1] / closest_line[0]

    return offset_x


def draw_lines(frame, lines):
    if lines is not None:
        for line in lines:
            rho, theta = line[0][0], line[0][1]
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a * rho
            y0 = b * rho
            x1 = int(x0 + 1000 * -b)
            y1 = int(y0 + 1000 * a)
            x2 = int(x0 - 1000 * -b)
            y2 = int(y0 - 1000 * a)
            cv2.line(frame, (x1, y1), (x2, y2), (150, 255, 0), 2)

# class LineFollower():
#     def __init__(self):
#         self.num_lines = 10
#
#     def update(self, frame, draw_frame=None):
#         frame_lines = cv2.Canny(frame, 1, 100)
#         # smooth the edge detection image for easier use(above)
#
#         lines = cv2.HoughLines(frame_lines, rho=1, theta=np.pi / 180,
#                                threshold=100, )
#         #    min_theta=-60 * np.pi / 180,
#         #    max_theta= 60 * np.pi / 180)
#         if lines is not None and len(lines) >= self.num_lines:
#             for line in lines[0:self.num_lines]:
#                 for rho, theta in line:
#                     a = np.cos(theta)
#                     b = np.sin(theta)
#                     x0 = a * rho
#                     y0 = b * rho
#                     x1 = int(x0 + 1000 * (-b))
#                     y1 = int(y0 + 1000 * (a))
#                     x2 = int(x0 - 1000 * (-b))
#                     y2 = int(y0 - 1000 * (a))
#
#                     if draw_frame is None:
#                         cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
#                     else:
#                         cv2.line(draw_frame, (x1, y1), (x2, y2), (0, 0, 255), 2)

# class LineFollower:
#     def __init__(self, expected, y_bottom, width, height):
#         self.centerRho, self.centerTheta = expected
#         self.width, self.height = width, height
#         self.yBottom = y_bottom
#
#     def isEqual(self, currentTheta, existedTheta, tolerance):
#         '''
#         if rho and theta are close enough,
#         set these lines as equivalent
#         To minimize # of lines on screen
#         '''
#         if abs(currentTheta - existedTheta) <= tolerance:
#             return True
#         return False
#
#     def merge(self, line_set):
#         occurance = len(line_set)
#         # if occurs more than once,
#         # merge and return a single median (rho, theta)
#         if occurance > 1:
#             medianTheta = np.median(line_set[0][0])
#             medianRho = np.median(line_set[0][1])
#             line_set = [occurance, medianRho, medianTheta]
#         else:
#             line_set = [occurance, line_set[0][0], line_set[0][1]]
#         return line_set
#
#     def findAverageLines(self, lines):
#         '''
#         findAvgLines is not supposed to draw;
#         use update to blur and find lines,
#         then use findAvglines func to return avgline
#         '''
#         rightRho, rightTheta, leftRho, leftTheta = [], [], [], []
#
#         # Divide lines into left and right groups, accoridng to sign of gradient
#         for currentLine in lines:
#             # notes on indexing: currentline has format[[x1, y1]]
#             (rho, theta)  = (currentLine[0], currentLine[1])
#             if theta > 0:
#                 # lines with negative gradient; (y increases downwards in frame)
#                 leftTheta.append(theta)
#                 leftRho.append(rho)
#
#             elif theta <= 0:
#                 rightTheta.append(theta)
#                 rightRho.append(rho)
#
#         if len(leftRho) != 0:
#             avgLeftRho = np.median([leftRho])
#             avgLeftTheta = np.median([leftTheta])
#         else:
#             (avgLeftRho, avgLeftTheta) = (0, 0)
#
#         if len(rightRho) != 0:
#             avgRightRho = np.median([rightRho])
#             avgRightTheta = np.median([rightTheta])
#         else: (avgRightRho, avgRightTheta) = (0, 0)
#
#         self.avgCenterRho = (avgLeftRho + avgRightRho) / 2.0
#         self.avgCenterTheta = (avgLeftTheta + avgRightTheta) / 2.0
#
#         return [(avgLeftRho, avgLeftTheta), (avgRightRho, avgRightTheta)]
#
#     def findLineCoord(self, rho, theta):
#         # turn avgLines into avgLinesCoord =[(x1, y1), (x2, y2)]
#         a = np.cos(theta)
#         b = np.sin(theta)
#         x0 = a * rho
#         y0 = b * rho
#         x1 = int(x0 + 1000 * -b)
#         y1 = int(y0 + 1000 * a)
#         x2 = int(x0 - 1000 * -b)
#         y2 = int(y0 - 1000 * a)
#         return (x1, y1, x2, y2)
#
#     def findDistLine(self,rho,theta):
#         (x1,y1,x2,y2) = self.findLineCoord(rho,theta)
#         Xavg = (x1+x2)/2
#         Yavg = (y1+y2)/2
#
#
#         return (self.width-Xavg,self.height-Yavg)
#
#     def key(self, item):
#         return item[0]
#
#     def difference(self, expected, actual, y_bottom):
#         return 0, 0  # distance difference, theta difference
#         ''' need to filter out unneeded lines before taking avg'''
#
#     def update(self, frame, draw_frame=None, draw_avg=True, draw_all=True,
#         maxNumLines = 10, tolerance = 0.04):
#         # frame = frame[90:360,::]
#         # frame_lines = cv2.medianBlur(frame, 5)
#         frame_lines = cv2.Canny(frame, 1, 100)
#         #smooth the edge detection image for easier use(above)
#
#         lines = cv2.HoughLines(frame_lines, rho=1, theta=np.pi / 180,
#                                threshold=100,
#                                min_theta=-60 * np.pi / 180,
#                                max_theta= 60 * np.pi / 180)
#
#
#         linesDrawn = []
#         # updating lines, after merge in similar ones
#         # condense lines together (remove "duplicates")
#         if lines != None:
#             lines = lines[:,0] # removing one layer of brackets
#
#             '''
#             tests on merging and sorting starts here,
#             1) lines are sorted accoriding to their rho value (len)
#             (could also sort according to theta)
#             2) while loops compare neighboring ones to partition them,
#             3) merge func also append multiplicity/ occurance of that partition
#             4) all lines are sorted based on # of occurance
#
#             '''
#             lines.sort(axis = 0) #sort on rho
#             print("LINES HERE----", lines)
#             i = -1 #used in loop
#             print(len(lines) // 2 - 1)
#             while i < (len(lines) // 2 - 1):
#                 #  len(lines) doublecounts (rho, theta)
#                 i += 1
#                 temp = []
#                 temp.append(np.array(lines[i]))
#                 while self.isEqual(lines[i][1], lines[i+1][1], tolerance):
#                     # ugly syntax, but it's comparing neighboring theta vals
#                     temp.append(lines[i+1])
#                     i += 1
#                 temp = self.merge(temp)
#                 linesDrawn.append(temp)
#
#             linesDrawn = np.array(linesDrawn)
#
#             #print (len(linesDrawn), "number of lines after merge") #for information purposes
#
#
#         #Sort the lines by distance from center by the average lines
#         temp = []
#         if lines != None:
#             #makes a list with dist from center included
#             for i in range(len(linesDrawn)):
#                 (rho, theta) = (linesDrawn[i][1], linesDrawn[i][2])
#                 dist = self.findDistLine(rho,theta)
#                 temp.append((dist,rho,theta))
#
#             #now want to sort this list based on dist
#             sorted_list = sorted(temp, key=self.key)
#             sorted_list = np.array(sorted_list)
#
#             #now draw the lines
#             if draw_all == True:
#                 idx = 0
#                 while idx < (maxNumLines/2 -1) and idx < len(sorted_list):
#                     (dist, rho, theta) = linesDrawn[idx]
#                     a = np.cos(theta)
#                     b = np.sin(theta)
#                     x0 = a * rho
#                     y0 = b * rho
#                     x1 = int(x0 + 1000 * -b)
#                     y1 = int(y0 + 1000 * a)
#                     x2 = int(x0 - 1000 * -b)
#                     y2 = int(y0 - 1000 * a)
#
#                     if draw_frame is None:
#                         cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
#                     else:
#                         cv2.line(draw_frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
#                     idx +=1
#
#
#
#         if lines is not None:
#
#             averaged_line = self.findAverageLines(lines[:maxNumLines])
#             (rho1, theta1) = (averaged_line)[0]
#             (rho2, theta2) = (averaged_line)[1]
#             (x1, y1, x2, y2) = self.findLineCoord(rho1, theta1)
#             (x3, y3, x4, y4) = self.findLineCoord(rho2, theta2)
#
#             # get coordinates of lines before drawing
#             if draw_avg:
#                 cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
#                 cv2.line(frame, (x3, y3), (x4, y4), (0, 255, 0), 2)
#
#         else:
#             averaged_line = None, None
#         return frame, self.difference((self.centerRho, self.centerTheta
#                                        ), averaged_line, self.yBottom)
