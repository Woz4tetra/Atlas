import bisect

import cv2
import numpy as np


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


def equalize_image(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    channels = cv2.split(hsv)
    # cv2.equalizeHist(channels[0], channels[0])
    # cv2.equalizeHist(channels[1], channels[1])
    cv2.equalizeHist(channels[2], channels[2])

    hsv = cv2.merge(channels)

    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)


def threshold_dark_image(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    gray = cv2.equalizeHist(gray)
    # gray = cv2.GaussianBlur(gray, (7, 7), 0)
    gray = cv2.medianBlur(gray, 11)

    thresh_val, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_OTSU)
    return binary


def get_contours(binary, epsilon=None, contour_num=-1):
    """
    Find the contours of an image and return an array of them sorted by increasing perimeter size.

    :param binary: The binary image that contours are to be calculated
    :param epsilon: If specified the approxPolyDP algorithm will be applied to the result.
                    Recommended value is 0.001
    :param contour_num: number of contours to returns. Picks the largest
                        contours by area
    :return: A 2D numpy array of the contours
    """
    binary, contours, hierarchy = cv2.findContours(binary.copy(), cv2.RETR_TREE,
                                                   cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    sig_contours = []
    perimeters = []

    for contour in contours[0:contour_num]:
        perimeter = cv2.arcLength(contour, closed=True)
        index = bisect.bisect(perimeters, perimeter)
        if epsilon is not None:
            approx = cv2.approxPolyDP(contour, epsilon * perimeter, False)
            sig_contours.insert(index, approx)
        else:
            sig_contours.insert(index, contour)
        perimeters.insert(index, perimeter)

    return sig_contours


def draw_contours(frame, contours):
    return cv2.drawContours(frame.copy(), contours, -1, (0, 0, 255), 1)


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


class Pipeline:
    def __init__(self, width, height, enable_draw):
        self.width = width
        self.height = height
        self.enable_draw = enable_draw

    def update(self, capture):
        # lines, frame = detect_lines(capture.frame, False)
        # draw_lines(frame, lines)

        return draw_contours(
            capture.frame,
            get_contours(
                threshold_dark_image(capture.frame), 0.01, 3
            )
        )
