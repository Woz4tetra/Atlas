import cv2
import numpy as np


class MeanShiftTracker:
    def __init__(self, frame):
        self.column = 125
        self.row = 100
        self.width = 80
        self.height = 90
        self.track_window = (self.column, self.row, self.width, self.height)

        roi = frame[self.row: self.row + self.height,
              self.column: self.column + self.width]
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_roi, np.array((0., 30., 32.)),
                           np.array((180., 255., 255.)))
        self.roi_hist = cv2.calcHist([hsv_roi], [0], mask, [180], [0, 180])
        cv2.normalize(self.roi_hist, self.roi_hist, 0, 255, cv2.NORM_MINMAX)
        self.term_crit = (
            cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 80, 1)

    def update(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        dst = cv2.calcBackProject([hsv], [0], self.roi_hist, [0, 180], 1)
        success, self.track_window = cv2.meanShift(dst, self.track_window,
                                                   self.term_crit)
        self.column, self.row, self.width, self.height = self.track_window
        cv2.rectangle(frame, (self.column, self.row),
                      (self.column + self.width, self.row + self.height), 255,
                      2)
        cv2.putText(frame, 'Tracked', (self.column - 25, self.row - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        return frame


class Pipeline:
    def __init__(self, width, height, enable_draw):
        self.width = width
        self.height = height
        self.enable_draw = enable_draw

    def update(self, capture, frame):
        return frame, {}
