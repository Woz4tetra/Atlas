from queue import Queue
from threading import Event, Thread

import cv2
import numpy as np

class Pipeline2:

    threshold = 70
    calibration_frame = 3058

    def __init__(self, camera, separate_read_thread=True):
        self.camera = camera
        self.status = None
        self.timestamp = None
        self.exit_event = Event()
        self._updated = False
        self.paused = False
        self.pause_updated = False
        self.frame = None

        # Filter for validating line
        self.width = None
        self.filter = np.random.rand(5,5,3) # creates a 5x5 filter for each R,G,B map
        self.on_screen = None # tells whether the desired line is on the screen
        self.moving_up = None # gives amount the percentage is changing, aim to get this to 0 for smooth change
        self.frame_counter = 0 # counts frames to help in filter creation
        # frame 3058 shows the desired line

        # Other
        self.prev_mid = None

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
                if not self.paused:
                    if self.separate_read_thread:
                        if self.frame_queue.empty():
                            continue

                        while not self.frame_queue.empty():
                            frame = self.frame_queue.get()

                            if frame is None:
                                self.status = "exit"
                                break

                            if self.width is None:
                                self.width = frame.shape[1]

                            self.frame = self.pipeline(frame)
                    else:
                        if self.camera.get_frame(self.timestamp) is None:
                            self.status = "exit"
                            break

                        self.frame = self.pipeline(self.camera.frame)

                    self.camera.show_frame(self.frame)
                    self._updated = True

                key = self.camera.key_pressed()
                if key == 'q':
                    self.close()
                elif key == ' ':
                    self.paused = not self.paused
                    self.pause_updated = True

        except BaseException as error:
            self.status = "error"
            thread_error = error

        self.camera.close()

        if thread_error is not None:
            raise thread_error

    def did_pause(self):
        if self.pause_updated:
            self.pause_updated = False
            return True
        else:
            return False

    def filter_init(self, frame):
        # allocate a 5x5 area to be a filter
        filter_frame = frame[(mid[0]-2):(mid[0]+2)][(mid[1]-2):(mid[1]+2)]



    def pipeline(self, frame):
        self.frame_counter += 1  # counts frames

        self.prev_safe_value = self.safety_value
        frame, lines, self.safety_value = self.hough_detector(frame.copy(), True)

        if self.frame_counter == 3058:
            self.filter_init(frame)

        # allows us to find the frame we want
        print(self.frame_counter)

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

        # if day_mode:
        #     output_frame = np.concatenate((output_frame, blur), axis=1)
        # else:
        #     output_frame = np.concatenate((output_frame, cv2.cvtColor(blur, cv2.COLOR_GRAY2BGR)), axis=1)

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

            return largest_y / height

        return 0.0

    def run_filter(self, frame, filter):
        mid = self.prev_mid


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

        # if self.prev_coords == None:
        #     self.prev_coords = [left_point, right_point]
        # elif self.is_valid_point(frame, coords, Pipeline2.threshold):
        #     self.prev_coords = [left_point, right_point]
        self.prev_mid = ((left_point[0]+right_point[0])/2, (left_point[1]+right_point[1])/2)

        print([left_point, right_point])

        cv2.circle(frame, left_point, 3, (255, 255, 255), 2)
        cv2.circle(frame, right_point, 3, (255, 255, 255), 2)

    # def is_valid_point(self, frame, coords, threshold):
    #     val = self.run_filter(frame, self.filter)
    #     if val < threshold:
    #         return True
    #     return False

    def close(self):
        self.exit_event.set()
        self.status = "done"