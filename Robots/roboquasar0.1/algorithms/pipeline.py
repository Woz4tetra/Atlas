import bisect
import time
from queue import Queue
from threading import Event, Thread

import cv2
import numpy as np


class Pipeline:
    def __init__(self, camera, day_mode, separate_read_thread=True):
        self.camera = camera
        self.status = None
        self.timestamp = None
        self.exit_event = Event()
        self._updated = False
        self.paused = False
        self.pause_updated = False
        self.frame = None
        self.day_mode = day_mode
        self.last_detection_t = time.time()

        self.hough_threshold = 95
        self.safety_threshold = 0.1
        self.safety_value = 0.0
        self.line_angle = 0.0
        self.prev_safe_value = 0.0
        self.safety_colors = ((0, 0, 255), (255, 113, 56), (255, 200, 56), (255, 255, 56), (208, 255, 100),
                              (133, 237, 93), (74, 206, 147), (33, 158, 193), (67, 83, 193), (83, 67, 193),
                              (160, 109, 193))

        # self.safety_colors = self.safety_colors[::-1]

        self.frame_queue = Queue(maxsize=255)

        self.separate_read_thread = separate_read_thread

        self.pipeline_thread = Thread(target=self.run)
        self.read_thread = Thread(target=self.read_camera)

    def start(self):
        if self.camera.enabled:
            if self.separate_read_thread:
                self.read_thread.start()
            self.pipeline_thread.start()

    def update_time(self, timestamp):
        self.timestamp = timestamp

    def did_update(self):
        if self._updated:
            self._updated = False
            return True
        else:
            return False

    def read_camera(self):
        thread_error = None
        try:
            while not self.exit_event.is_set():
                if not self.paused:
                    if self.camera.get_frame(self.timestamp) is None:
                        self.status = "exit"
                        break

                    if not self.frame_queue.full():
                        self.frame_queue.put(self.camera.frame)

        except BaseException as error:
            self.status = "error"
            thread_error = error

        if thread_error is not None:
            raise thread_error

    def run(self):
        thread_error = None
        try:
            while not self.exit_event.is_set():
                self.status = self._update()
                if self.status is not None:
                    break

        except BaseException as error:
            self.status = "error"
            thread_error = error

        print("Camera '%s' closing" % self.camera.name)
        self.camera.close()

        if thread_error is not None:
            raise thread_error

    def _update(self):
        if not self.paused:
            if self.separate_read_thread:
                if self.frame_queue.empty():
                    return

                while not self.frame_queue.empty():
                    frame = self.frame_queue.get()
                    if frame is None:
                        return "exit"

                    self.frame = self.pipeline(frame)
            else:
                if self.camera.get_frame(self.timestamp) is None:
                    self.status = "exit"
                    return "exit"

                self.frame = self.pipeline(self.camera.frame)

            self.camera.show_frame(self.frame)
            self._updated = True

        key = self.camera.key_pressed()
        if key == 'q':
            self.close()
            return "done"
        elif key == ' ':
            self.paused = not self.paused
            self.pause_updated = True

    def did_pause(self):
        if self.pause_updated:
            self.pause_updated = False
            return True
        else:
            return False

    def pipeline(self, frame):
        self.prev_safe_value = self.safety_value
        frame, lines, safety_value, line_angle = self.hough_detector(frame.copy(), self.day_mode)
        if safety_value != 0.0 or time.time() - self.last_detection_t > 2:
            self.safety_value = safety_value
            self.line_angle = line_angle
            self.last_detection_t = time.time()

        frame[10:40, 20:90] = self.safety_colors[int(self.safety_value * 10)]
        cv2.putText(frame, "%0.1f%%" % (self.safety_value * 100), (30, 30), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 0))

        # frame = self.kernel_threshold(self.camera.frame)
        # contours, perimeters = self.get_contours(frame, 0.025, 2)
        # if len(contours) > 0:
        #     frame = self.draw_contours(self.camera.frame, contours)
        #
        #     frame, results = self.get_pavement_edge(perimeters, contours, frame)
        #
        # frame = self.average_color(frame)

        return frame

    def get_pavement_edge(self, perimeters, contours, frame):
        results = []
        if perimeters[-1] > 2000:
            contour = contours[-1]
            for index in range(1, len(contour)):
                point1 = contour[index - 1][0]
                point2 = contour[index][0]
                diff = point2 - point1
                angle = np.arctan2(diff[0], diff[1])
                if -np.pi / 2 - 0.5 < angle < -np.pi / 2 + 0.5:
                    frame = cv2.line(frame, tuple(point1.tolist()), tuple(point2.tolist()), (50, 200, 0), thickness=10)
                    results.append((angle, point1, point2))
        return frame, results

    def average_color(self, frame):
        avg_color = np.uint8(np.average(np.average(frame, axis=0), axis=0))
        height, width = self.camera.frame.shape[0:2]
        avg_color_top = self.average_color(frame[:height // 2])
        avg_color_bot = self.average_color(frame[height // 2:])

        frame[0:height // 8, 0:width // 4, :] = avg_color_top
        frame[height // 8:height // 4, 0:width // 4, :] = avg_color_bot
        frame = cv2.putText(frame, str(avg_color_top)[1:-1], (0, height // 8 - 5), cv2.FONT_HERSHEY_PLAIN,
                            1, (255, 255, 255))
        frame = cv2.putText(frame, str(avg_color_bot)[1:-1], (0, height // 4 - 5), cv2.FONT_HERSHEY_PLAIN,
                            1, (255, 255, 255))
        return avg_color, frame

    def kernel_threshold(self, frame):
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame = cv2.equalizeHist(frame)

        value, frame = cv2.threshold(frame, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

        # noise removal
        kernel = np.ones((3, 3), np.uint8)
        opening = cv2.morphologyEx(frame, cv2.MORPH_OPEN, kernel, iterations=2)

        # sure background area
        sure_bg = cv2.dilate(opening, kernel, iterations=3)

        # Finding sure foreground area
        dist_transform = cv2.distanceTransform(opening, cv2.DIST_L2, 5)
        ret, sure_fg = cv2.threshold(dist_transform, 0.1 * dist_transform.max(), 255, 0)

        return dist_transform

        # Finding unknown region
        # sure_fg = np.uint8(sure_fg)
        # unknown = cv2.subtract(sure_bg, sure_fg)

        # Marker labelling
        # ret, markers = cv2.connectedComponents(sure_fg)
        #
        # # Add one to all labels so that sure background is not 0, but 1
        # markers = markers + 1
        #
        # # Now, mark the region of unknown with zero
        # markers[unknown == 255] = 0
        #
        # markers = cv2.watershed(self.camera.frame, markers)
        # # self.camera.frame[markers == -1] = [255, 0, 0]
        #
        # blank = np.ones(self.camera.frame.shape[0:2], dtype=np.uint8) * 255
        # blank[markers == -1] = 0

    def hough_detector(self, input_frame, day_mode):
        # blur = cv2.medianBlur(input_frame, 5)
        if day_mode:
            # self.hough_threshold = 150
            blur = cv2.cvtColor(input_frame, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(blur, (15, 15), 0)
            # blur = cv2.GaussianBlur(blur, (5, 5), 0)
        else:
            # self.hough_threshold = 125
            blur = cv2.cvtColor(input_frame, cv2.COLOR_BGR2GRAY)
            blur = cv2.equalizeHist(blur)
            blur = cv2.GaussianBlur(blur, (7, 7), 0)

        frame = cv2.Canny(blur, 1, 100)
        lines = cv2.HoughLines(frame, rho=1.2, theta=np.pi / 180,
                               threshold=self.hough_threshold,
                               min_theta=60 * np.pi / 180,
                               max_theta=120 * np.pi / 180
                               )
        safety_percentage, line_angle = self.draw_lines(input_frame, lines)

        output_frame = cv2.add(cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR), input_frame)
        # if day_mode:
        #     output_frame = np.concatenate((output_frame, blur), axis=1)
        # else:
        output_frame = np.concatenate((output_frame, cv2.cvtColor(blur, cv2.COLOR_GRAY2BGR)), axis=1)
        return output_frame, lines, safety_percentage, line_angle

    def sobel_filter(self, frame):
        # frame = cv2.Laplacian(frame, cv2.CV_64F, ksize=3)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame = cv2.medianBlur(frame, 3)
        frame = cv2.Sobel(frame, cv2.CV_64F, 0, 1, ksize=3)

        frame = np.absolute(frame)
        frame = np.uint8(frame)
        return frame

    def threshold(self, input_frame):
        frame = cv2.cvtColor(input_frame, cv2.COLOR_BGR2GRAY)
        frame = cv2.medianBlur(frame, 11)
        frame = cv2.equalizeHist(frame)

        thresh_val, frame = cv2.threshold(frame, 0, 255, cv2.THRESH_OTSU)
        # thresh_val, frame = cv2.threshold(frame, 100, 255, cv2.THRESH_BINARY)
        return frame

    def draw_contours(self, frame, contours):
        return cv2.drawContours(frame.copy(), contours, -1, (0, 0, 255), 3)

    def get_contours(self, binary, epsilon=None, num_contours=None):
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

        if num_contours is None:
            return sig_contours, perimeters
        else:
            return sig_contours[-num_contours:], perimeters[-num_contours:]

    def draw_lines(self, frame, lines, draw_threshold=30):
        height, width = frame.shape[0:2]

        if lines is not None:
            counter = 0
            largest_y = 0
            left_y = 0
            right_y = 0
            largest_coords = None

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

                y3 = int(y0 - width / 2 * a)

                # if y2 > largest_y:
                #     largest_y = y2
                #     largest_coords = (x1, y1), (x2, y2)
                if y3 > largest_y:
                    largest_y = y3
                    largest_coords = x1, y1, x2, y2
                    left_y = y0
                    right_y = int(y0 - width * a)

                cv2.line(frame, (x1, y1), (x2, y2), (150, 150, 150), 2)
                # cv2.circle(frame, (x0, y0), 10, (150, 255, 50), 2)
                counter += 1
                if counter > draw_threshold:
                    break

            if largest_coords is not None:
                x1, y1, x2, y2 = largest_coords
                cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                line_angle = np.arctan2(y1 - y2, x1 - x2)
                if line_angle < 0:
                    line_angle += 2 * np.pi
                line_angle -= np.pi
                line_angle *= -1
            else:
                line_angle = 0.0

            if left_y > right_y:
                safety_value = left_y / height
            else:
                safety_value = right_y / height

            if safety_value > 1.0:
                safety_value = 1.0
            if safety_value < 0.0:
                safety_value = 0.0
            # print("%0.4f, %0.4f" % (safety_value, line_angle))
            # time.sleep(0.01)
            return safety_value, line_angle
            # return largest_y / height
        return 0.0, 0.0

    def close(self):
        self.exit_event.set()
        self.status = "done"
        print("Closing pipeline")


class PID:
    def __init__(self, kp, kd, ki, lower_limit, upper_limit):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.prev_value = None
        self.prev_error = 0
        self.error_sum = 0
        self.prev_time = 0
        self.lower_limit = lower_limit
        self.upper_limit = upper_limit

    def update(self, value, goal, timestamp):
        if self.prev_value is None:
            self.prev_value = value
            return 0.0

        dt = timestamp - self.prev_time

        error = goal - value
        deriv = (error - self.prev_error) / dt
        self.error_sum += error * dt

        self.prev_value = value
        self.prev_error = error
        self.prev_time = timestamp

        output = self.kp * error + self.kd * deriv + self.ki * self.error_sum
        if output > self.upper_limit:
            output = self.upper_limit
        if output < self.lower_limit:
            output = self.lower_limit
        return output
