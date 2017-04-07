from queue import Queue
from threading import Event, Thread
from algorithms.convnet import NeuralNetwork

import cv2
import numpy as np
import time
import math

class Pipeline2:
    threshold = 70
    # calibration_frame = 3053

    # calibration_frame = 725
    calibration_frame = 495

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

        # Filter for validating line
        self.network = None
        self.train_shape = (21,21,3)
        self.network_on = True

        # self.on_screen = None  # tells whether the desired line is on the screen
        # self.moving_up = None  # gives amount the percentage is changing, aim to get this to 0 for smooth change
        self.frame_counter = 0  # counts frames to help in filter creation
        self.prev_mid = None
        self.prev_coords = None

        self.safety_threshold = 0.1
        self.safety_value = 0.0
        self.prev_safe_value = 0.0
        self.safety_colors = ((0, 0, 255), (255, 113, 56), (255, 200, 56), (255, 255, 56), (208, 255, 100),
                              (133, 237, 93), (74, 206, 147), (33, 158, 193), (67, 83, 193), (83, 67, 193),
                              (160, 109, 193))
        # self.safety_colors = self.safety_colors[::-1]

        self.frame_queue = Queue(maxsize=255)

        self.separate_read_thread = separate_read_thread

        self.pipeline_thread = Thread(target=self.run)
        self.read_thread = Thread(target=self.read_camera)

    def preprocess_frame(self, frame):
        # get slope from previous coords
        left = self.prev_coords[0]
        right = self.prev_coords[1]
        slope = (right[1]-left[1])/(right[0]-left[0])
        frames_and_labels = []

        frame_altered = frame.astype(np.float32)

        # make a bunch of correct edge detections
        L = int(self.train_shape[0] / 2)
        R = L+1

        for i in range(L, self.camera.width - R):
            # create 7x7 frame with label 1 (correct edge)
            if L <= i*slope <= self.camera.height - R:
                if frame_altered[i-L: i+R, int(i*slope)-L: int(i*slope)+R].shape == self.train_shape:
                    frame_and_label = [self.normalize_frame(frame_altered[i-L: i+R, int(i*slope)-L: int(i*slope)+R]), 1]
                    frames_and_labels.append(frame_and_label)

            # create 7x7 frame with label 0 (incorrect edge)
            if i*slope + R <= 2 * self.train_shape[1]:
                if frame_altered[i - L: i + R, self.camera.height - self.train_shape[1]: self.camera.height].shape == self.train_shape:
                    frame_and_label = [self.normalize_frame(frame_altered[i - L: i + R, self.camera.height - self.train_shape[1]: self.camera.height]), 0]
                    frames_and_labels.append(frame_and_label)

            else:
                if frame_altered[i - L: i + R, 0: self.train_shape[1]].shape == self.train_shape:
                    frame_and_label = [self.normalize_frame(frame_altered[i - L: i + R, 0: self.train_shape[1]]), 0]
                    frames_and_labels.append(frame_and_label)

        return frames_and_labels

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
                # self.frame = self.camera.frame

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

    def get_center_frames(self, frame):
        left = self.prev_coords[0]
        right = self.prev_coords[1]
        slope = (right[1] - left[1]) / (right[0] - left[0])

        frame_altered = frame.astype(np.float32)

        frames = []

        L = int(self.train_shape[0] / 2)
        R = int(self.train_shape[0] - L)

        for i in range(L, self.camera.width - R):
            # create 7x7 frame with label 1 (correct edge)
            if L <= i * slope <= self.camera.height - R:
                if frame_altered[i - L: i + R, int(i * slope) - L: int(i * slope) + R].shape == self.train_shape:
                    frame_new = [self.normalize_frame(frame_altered[i - L: i + R, int(i * slope) - L: int(i * slope) + R])]
                    frames.append(frame_new)

        return frames

    def normalize_frame(self, frame, method=1):
        # requires a 7x7x3 frame and will return a normalized frame
        filter_frame = np.zeros(list(self.train_shape))

        if method == 1:
            for c in range(3):
                max_v = np.max(frame[:, :, c])
                min_v = np.min(frame[:, :, c])
                # filter_frame[:, :, c] = (frame[:, :, c] - min_v) / (max_v - min_v)
                filter_frame[:,:,c] = frame[:,:,c] / 255

        return filter_frame

    def pipeline(self, frame):
        self.frame_counter += 1  # counts frames

        self.prev_safe_value = self.safety_value
        frame, lines, safety_value = self.hough_detector(frame.copy(), self.day_mode)

        if not self.network_on:
            print(self.frame_counter)

        if safety_value != 0.0 or time.time() - self.last_detection_t > 2:
            self.safety_value = safety_value
            self.last_detection_t = time.time()

        if self.frame_counter == Pipeline2.calibration_frame and self.network_on:
            frames_and_labels = self.preprocess_frame(frame)
            self.network = NeuralNetwork(frames_and_labels, self.train_shape)

        frame[10:40, 20:90] = self.safety_colors[int(self.safety_value * 10)]
        cv2.putText(frame, "%0.1f%%" % (self.safety_value * 100), (30, 30), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 0))

        return frame

    def hough_detector(self, input_frame, day_mode):
        # blur = cv2.medianBlur(input_frame, 5)
        if day_mode:
            blur = cv2.GaussianBlur(input_frame, (11, 11), 0)
        else:
            blur = cv2.cvtColor(input_frame, cv2.COLOR_BGR2GRAY)
            blur = cv2.equalizeHist(blur)
            blur = cv2.GaussianBlur(blur, (7, 7), 0)

        frame = cv2.Canny(blur, 1, 100)
        lines = cv2.HoughLines(frame, rho=1.0, theta=np.pi / 180,
                               threshold=125,
                               min_theta=80 * np.pi / 180,
                               max_theta=110 * np.pi / 180
                               )
        safety_percentage = self.draw_lines(input_frame, lines)

        output_frame = cv2.add(cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR), input_frame)

        if day_mode:
            output_frame = np.concatenate((output_frame, blur), axis=1)
        else:
            output_frame = np.concatenate((output_frame, cv2.cvtColor(blur, cv2.COLOR_GRAY2BGR)), axis=1)

        return output_frame, lines, safety_percentage

    def draw_lines(self, frame, lines, draw_threshold=30):
        height, width = frame.shape[0:2]

        if lines is not None:
            counter = 0
            largest_y = 0
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

                if y3 > largest_y:
                    largest_y = y3
                    largest_coords = (x1, y1), (x2, y2)

                cv2.line(frame, (x1, y1), (x2, y2), (150, 150, 150), 2)

                counter += 1
                if counter > draw_threshold:
                    break

            if largest_coords is not None:
                cv2.line(frame, largest_coords[0], largest_coords[1], (0, 0, 255), 2)
                self.find_borderpoints(frame, largest_coords)
                if self.network != None and self.network_on:
                    working_frames = self.get_center_frames(frame)
                    print(self.network.run_network(np.float32(working_frames))[0][0])

            return largest_y / height

        return 0.0

    def find_borderpoints(self, frame, coords):
        height, width = frame.shape[0:2]
        p1 = coords[0]
        p2 = coords[1]

        # find the slope between the points
        slope = (p2[1] - p1[1]) / (p2[0] - p1[0])

        # find the left-most point
        left_point = (0,
                      int(p1[1] + slope * (-p1[0])))
        # find the right-most point
        right_point = (width,
                       int(p1[1] + slope * (width - p1[0])))

        self.prev_mid = (int((left_point[0] + right_point[0]) / 2), int((left_point[1] + right_point[1]) / 2))
        self.prev_coords = (left_point, right_point)

        cv2.circle(frame, left_point, 3, (255, 255, 255), 2)
        cv2.circle(frame, right_point, 3, (255, 255, 255), 2)

    def close(self):
        self.exit_event.set()
        self.status = "done"
