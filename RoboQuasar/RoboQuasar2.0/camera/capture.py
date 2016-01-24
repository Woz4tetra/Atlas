# handles all interfacing with OpenCV camera objects

import os
import time
import datetime
import sys

import cv2
import numpy
import config


class Capture(object):
    excludedSources = set()
    increaseFPSActions = []

    resolutions = {
        "logitech": {
            8: (1920, 1080),
            12: (1440, 810),
            23: (960, 540),
            30: (480, 270),
            31: (240, 135),
            "default": 30
        },
        "ELP": {
            10: (2048, 1536),
            12: (1600, 1200),
            30: (1280, 720),
            31: (640, 480),
            32: (320, 240),
            "default": 30
        },
        "ELP180": {
            6: (1920, 1080),
            9: (1280, 720),
            21: (800, 600),
            30: (640, 480),
            31: (352, 288),
            32: (320, 240),
            "default": 30
        },
        "ps3eye": {
            60: (640, 480),
            120: (320, 240),
            "default": 60
        }
    }

    mac_keys = {
        63232: "up",
        63233: "down",
        63234: "left",
        63235: "right",
        27: "esc",
        13: "enter"
    }

    linux_keys = {
        65362: "up",
        65364: "down",
        65361: "left",
        65363: "right",
        'esc': 'esc',
        10: "enter"
    }

    windows_keys = {

    }

    def __init__(self, window_name=None, cam_source=None,
                 auto_select_source=False,
                 show_window=True,
                 width=None, height=None, crop=(None,) * 4,
                 size_by_fps=None, camera_type=None,
                 frame_skip=0,
                 loop_video=True,
                 start_frame=0,
                 quit_on_end=True):
        """
        The initializer for an opencv camera object. It holds all the methods
        required to read and write from a camera.

        :param window_name: The name of the window to display the camera output.
                If None, False, or "" (Falsey), the camera will not show its
                output although you can still read data from it.
        :param cam_source: The camera number or the name of the video file.
                If a number, Capture will attempt
                cv2.VideoCapture(camSource). OpenCV's camera numbering system
                is arbitrary at best, so this isn't recommended.

                If None, Capture will search for a camera for you. If you'd like
                manually select a camera , set autoSelectSource to True
                (see that parameter for details)

                If a string, Capture will search for a video of that name in the
                directory "Camera/Videos"
        :param width: Manually set a width for the capture. Capture will scale
                the capture to this width if specified. I'd recommend sizeByFPS,
                however, as it optimizes size to desired FPS for you.

                Note: For videos, OpenCV's resize function is used. This can
                cause a significant drop in fps
        :param height: Same as width except for height
        :param size_by_fps: The desired FPS for the capture. The FPS for the
                following cameras are implemented under the following names:

                ELP180: "180degree Fisheye Lens 1080p Wide Angle Pc Web USB Camera"
                ELP: ELP USB with Camera 2.1mm Wide Angle Mjpeg 5megapixel Hd Camera USB for Industrial, Machine Vision
                logitech: Logitech HD Pro Webcam C920

                Type help(Capture.resolutions) for details.

                To specify which camera's data to use, specify the cameraType
                parameter.

                If you type an FPS that doesn't match the ones under
                Capture.resolutions, Capture will find the closest FPS.

                If None, Capture picks the default resolution for the logitech
                camera type: (480, 270).
        :param camera_type: Specify which camera data to use for sizeByFPS. If
                None, Capture selects logitech by default
        :param crop: How much to crop the resulting image by. Supply a tuple of
                size 4. If you don't want to crop a certain dimension, set that
                dimension to None.

                Example: (50, None, None, 100) crops the image to size
                            (50, 0, width, 100)
                         (None, 50, 100, None) crops the image to size
                            (0, 50, 100, height)
                         (None, None, None, None) crops the image to size
                            (0, 0, width, height)
        :param auto_select_source: If True, Capture will launch the Camera
                Selector mini application. Follow its instructions when prompted.
        :param frame_skip: Number of frames to skip in a video or camera feed.
                This works much better for videos than cameras.
        :param loop_video: Whether to loop a video when it ends or not
        """
        time1 = time.time()

        self.windowName = window_name
        self.camSource = cam_source
        self.enableDraw = show_window
        self.sizeByFPS = size_by_fps
        self.isRunning = True
        self.analysisApplied = False
        self.cameraType = camera_type
        self.cameraFPS = None
        self.trackbarName = "Frame"
        self.dimensions = list(crop)
        self.video = None
        self.loopVideo = loop_video
        self.quitOnEnd = quit_on_end

        if type(self.camSource) == int:
            self.frameSkip = 0
        else:
            self.frameSkip = frame_skip

        if sys.platform.startswith('darwin'):  # OS X
            self.key_codes = self.mac_keys
        elif (sys.platform.startswith('linux') or sys.platform.startswith(
                'cygwin')):
            self.key_codes = self.linux_keys
        elif sys.platform.startswith('win'):  # Windows
            self.key_codes = self.windows_keys
        else:
            raise EnvironmentError('Unsupported platform')

        camera_type = camera_type if camera_type is not None else "logitech"
        try:
            self.resolutions = Capture.resolutions[camera_type]
        except:
            raise Exception("camera data for camera type %s does not exist.\
Please type help(Capture.resolutions) for a dictionary of available camera data.")

        if self.enableDraw == True:
            cv2.namedWindow(window_name)

        if cam_source is not None:
            if type(cam_source) == str:
                self.loadVideo(cam_source)
            else:
                self.loadCamera(cam_source)
        else:
            if auto_select_source is True:
                capture = self.searchForCamera()
            else:
                capture = self.cameraSelector()
            self.loadCamera(capture)

        if width is not None and height is not None:
            self.width, self.height = width, height

        elif self.sizeByFPS is not None:
            if (self.sizeByFPS in list(self.resolutions.keys())) is False:
                self.sizeByFPS = self.findClosestRes(size_by_fps)
            self.width, self.height = self.resolutions[self.sizeByFPS]

        else:
            if type(self.camSource) == int:
                self.sizeByFPS = self.resolutions["default"]
                self.width, self.height = self.resolutions[self.sizeByFPS]

            else:
                self.width, self.height = self.camera.get(
                        cv2.CAP_PROP_FRAME_WIDTH), self.camera.get(
                        cv2.CAP_PROP_FRAME_HEIGHT)

        if type(self.camSource) == int:
            Capture.increaseFPSActions.append(self.increaseFPS)
            if len(Capture.increaseFPSActions) >= 2:
                for increaseFPS in Capture.increaseFPSActions:
                    increaseFPS()
                Capture.increaseFPSActions = []

        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        if self.dimensions[0] is None:
            self.dimensions[0] = 0
        if self.dimensions[1] is None:
            self.dimensions[1] = 0
        if self.dimensions[2] is None:
            self.dimensions[2] = self.width
        if self.dimensions[3] is None:
            self.dimensions[3] = self.height

        time2 = time.time()
        print((str(self.camSource) + " loaded in " + str(
                time2 - time1) + " seconds. Capture size is " +
              str(int(self.width)) + "x" + str(int(self.height))))

        if start_frame > 0:
            self.setFrame(start_frame)

        self.frame = numpy.zeros((self.height, self.width, 3))

    def cameraSelector(self):
        """
        A mini application to assist in camera selection. Cycle through the
        cameras with the left and right arrow keys and jump between cameras with
        the number pad. Press enter to confirm and q or esc to cancel.

        :return: The selected camera number
        """
        print("Welcome to the Camera Selector!\n\n")
        print("Use the arrow keys to switch between cameras.")
        print("Enter numbers on your number pad to jump to different cameras.")
        print("Press enter to confirm and q or esc to cancel.")

        shape = None
        windowName = "camera #"
        capture = 0

        def updateCapture(windowName, video, capture, delta=None,
                          newCapture=None):
            print(str(capture) + " ---> ", end=' ')

            video.release()

            cv2.destroyWindow(windowName + str(capture))
            if delta is not None:
                capture += delta
            elif newCapture is not None:
                capture = newCapture
            print(capture)

            video = cv2.VideoCapture(capture)

            video.set(cv2.CAP_PROP_FRAME_WIDTH, 720)
            video.set(cv2.CAP_PROP_FRAME_HEIGHT, 450)

            return video, capture

        video = cv2.VideoCapture(capture)

        video.set(cv2.CAP_PROP_FRAME_WIDTH, 720)
        video.set(cv2.CAP_PROP_FRAME_HEIGHT, 450)

        while True:
            key = self.getPressedKey(1)
            if key == "left":
                print("Loading camera. Please wait...")
                video, capture = updateCapture(windowName, video, capture,
                                               delta=-1)
            elif key == "right":
                print("Loading camera. Please wait...")
                video, capture = updateCapture(windowName, video, capture,
                                               delta=+1)
            elif type(key) == str and key.isdigit():
                video, capture = updateCapture(windowName, video, capture,
                                               newCapture=int(key))
            elif key == "enter":
                cv2.destroyWindow(windowName + str(capture))
                return capture
            elif key == 'q' or key == "esc":
                quit()

            success, frame = video.read()

            if success is True and frame is not None:
                cv2.imshow(windowName + str(capture), frame)
                shape = frame.shape
            else:
                cv2.imshow(windowName + str(capture), numpy.zeros(shape))
            
    def searchForCamera(self):
        """
        Searches for an available camera by trying all numbers between
        0...10 then -1...-10. It will ignored all captures in
        Capture.excludedSources.

        :return: The selected camera number
        """
        success, frame = False, None
        capture = 0
        while True:
            temp = cv2.VideoCapture(capture)
            success, frame = temp.read()
            if (success is True or capture > 10) and (
                        capture in Capture.excludedSources) == False:
                break
            capture += 1

        if success is False:
            capture = -1
            while True:
                temp = cv2.VideoCapture(capture)
                success, frame = temp.read()
                if (success is True or capture < -10) and (
                            capture in Capture.excludedSources) == False:
                    break
                capture -= 1
        if success is False:
            raise Exception("Camera could not be found")

        return capture

    def loadCamera(self, capture):
        """
        Loads a numbered camera and adds it to Capture.excludedSources so that
        duplicate cameras don't arise.

        :param capture: The camera number to load
        :return: None
        """
        print("loading camera " + str(capture) + " into window named '" + str(
                self.windowName) + "'...")
        self.camera = cv2.VideoCapture(capture)
        self.camSource = capture
        Capture.excludedSources.add(capture)

    def loadVideo(self, camSource):
        print("loading video into window named '" + str(
                self.windowName) + "'...")
        self.camera = cv2.VideoCapture(config.get_dir(":videos") + camSource)

        self.cameraFPS = self.camera.get(cv2.CAP_PROP_FPS)
        self.lenVideoFrames = int(self.camera.get(cv2.CAP_PROP_FRAME_COUNT))
        if self.lenVideoFrames <= 0:
            raise Exception(
                    "Video failed to load! Did you misspell the video name?")

        self.videoLength_sec = self.lenVideoFrames / self.cameraFPS
        self.singleFrame_sec = 1.0 / (self.cameraFPS * 1000)

        print("\tfps:", self.cameraFPS)
        print("\tlength (sec):", self.videoLength_sec)
        print("\tlength (frames):", self.lenVideoFrames)

        cv2.createTrackbar(self.trackbarName, self.windowName, 0,
                           # int(self.camera.get(cv2.CAP_PROP_FRAME_COUNT)),
                           self.lenVideoFrames, self.onSlider)

        print("video loaded!")

    def findClosestRes(self, sizeByFPS):
        """
        Finds the closest number in self.resolutions. self.resolutions is
        determined by the cameraType parameter. See Capture.__init__ for
        details.

        :param sizeByFPS: An integer specifying desired FPS
        :return: An integer closest to sizeByFPS in self.resolutions
        """
        possibleFPSs = numpy.array(list(self.resolutions.keys()))
        minuend = possibleFPSs.copy()
        minuend.fill(sizeByFPS)
        difference = possibleFPSs - minuend
        difference = numpy.absolute(difference)
        minimum = numpy.min(difference)
        index = numpy.where(difference == minimum)[0][0]
        return possibleFPSs[index]

    def stopCamera(self):
        """
        End an OpenCV capture cleanly and close the corresponding window

        :return: None
        """
        self.isRunning = False
        self.camera.release()
        self.stopVideo()
        cv2.destroyWindow(self.windowName)

    def setFrame(self, frameNumber):
        """
        If this Capture is a video, set the current frame to frameNumber.

        :param frameNumber: An integer specifying the desired frame
        :return: None
        """
        if frameNumber >= self.camera.get(cv2.CAP_PROP_FRAME_COUNT):
            frameNumber = 0
        if type(self.camSource) == str and frameNumber >= 0:
            self.camera.set(cv2.CAP_PROP_POS_FRAMES, frameNumber)

    def incrementFrame(self):
        """
        If this Capture is a video, increment the current frame by 1.

        :return: None
        """
        currentFrame = self.camera.get(cv2.CAP_PROP_POS_FRAMES)
        self.setFrame(currentFrame + 1.8)

    def decrementFrame(self):
        """
        If this Capture is a video, decrement the current frame by 1.

        :return: None
        """
        currentFrame = self.camera.get(cv2.CAP_PROP_POS_FRAMES)
        self.setFrame(currentFrame - 1.8)

    def saveFrame(self, frame=None, default_name=True, directory=None):
        """
        Write the input frame to Camera/Images

        :param frame: A numpy array containing the frame to write
                (shape = (height, width, 3))
        :return: None
        """
        if not default_name:
            name = time.strftime("%c").replace(":", ";") + ".png"
            print("Frame saved as " + str(name))
            print("in directory:\n" + config.get_dir(":images"))
        else:
            name = datetime.datetime.now().strftime(
                    "%a %b %d %H;%M;%S.%f %p, %Y") + ".png"

        if frame is None:
            frame = self.frame

        if directory == None:
            directory = config.get_dir(":images")

        if not os.path.isdir(directory):
            os.makedirs(directory)

        cv2.imwrite(directory + name, frame)

    def increaseFPS(self):
        """
        If another camera is added, Capture will automatically increase the
        FPS to maintain stability. This is done by selecting the next biggest
        entry in self.resolutions. self.resolutions is determined by the
        cameraType parameter. See Capture.__init__ for details.

        :return: None
        """
        possibleFPSs = sorted(self.resolutions.keys())
        index = possibleFPSs.index(self.sizeByFPS)
        if (index + 1) < len(possibleFPSs):
            index += 1

        self.sizeByFPS = possibleFPSs[index]
        self.width, self.height = self.resolutions[self.sizeByFPS]
        self.setCameraSize(self.width, self.height)

    def setCameraSize(self, width=None, height=None):
        """
        Set the capture width and height

        :param width: New capture width
        :param height: New capture height
        :return: None
        """
        if self.width != None:
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.width = width
        if self.height != None:
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.height = height

    def onSlider(self, frameIndex):
        """
        If Capture is a video, When the slider is changed, jump the capture to
        corresponding frame.

        :param frameIndex: Frame number to jump to
        :return: None
        """
        if frameIndex != self.currentFrameNumber():
            self.setFrame(frameIndex)
            self.showFrame(self.getFrame(False))

    def getPressedKey(self, delay=1):
        """
        Using OpenCV's waitKey, get the user's pressed key.

        :param delay: An optional delay (milliseconds) to wait for user input.
                Default is 1.
        :return: The pressed key as a single character string OR if the key
                number matches Capture.mac_keys, it will return the
                corresponding text. Type help(Capture.mac_keys) for details.
        """
        key = cv2.waitKey(delay)
        if key in self.key_codes:
            return self.key_codes[key]
        elif key > -1:
            if 0 <= key < 0x100:
                return chr(key)
            else:
                print(("Unrecognized key: " + str(key)))
        else:
            return key

    def showFrame(self, frame=None):
        """
        Display the frame in the Capture's window using cv2.imshow

        :param frame: A numpy array containing the image to be displayed
                (shape = (height, width, 3))
        :return: None
        """
        if frame is not None:
            cv2.imshow(self.windowName, frame)
        else:
            cv2.imshow(self.windowName, self.frame)


    def currentFrameNumber(self):
        """
        If this Capture is a video, return the Capture's frame number

        :return: the frame number (might not be an integer)
        """
        return int(self.camera.get(cv2.CAP_PROP_POS_FRAMES))

    def getVideoFPS(self):
        """
        If this Capture is a video, return the Capture's FPS

        NOTE: this does not work for videos.

        :return: the FPS
        """
        return self.camera.get(cv2.CAP_PROP_FPS)

    def initVideoWriter(self, fps=30, video_name=None, includeTimestamp=True,
                        codec='mp4v', format='mov',
                        output_dir=None):
        """
        Initialize the Capture's video writer.

        :param fps: The playback FPS of the video. This number can be finicky as
                the capture's current FPS may not match the video's output FPS.
                This is because video playback takes less computation than
                analyzing the video in this setting.
        :param video_name: The name of the video. If "", a time stamp will
                automatically be inserted
        :param includeTimestamp: An optional parameter specifying whether the
                time should be included. True by default
        :param codec: The output video codec. mp4v is recommended
        :return: None
        """
        if video_name == None:
            video_name = ""
        elif video_name != None and includeTimestamp == True:
            video_name += " "

        if includeTimestamp == True:
            video_name += time.strftime("%c").replace(":", ";") + "." + format

        if output_dir == None:
            output_dir = config.get_dir(":videos") + video_name
        else:
            output_dir += "/" + video_name

        if not os.path.isdir(output_dir):
            os.makedirs(output_dir)

        fourcc = cv2.VideoWriter_fourcc(*codec)
        self.video = cv2.VideoWriter()
        self.video.open(output_dir, fourcc, fps,
                        (int(self.dimensions[2] - self.dimensions[0]),
                         int(self.dimensions[3] - self.dimensions[1])), True)

        self.videoOutputDir = output_dir
        print("Initialized video named '%s'." % (video_name))

    def writeToVideo(self, frame):
        """
        Write the frame to the Capture's initialized video capture.
        Type help(Capture.initVideoWriter) for details.

        :param frame: A numpy array containing the frame to write
                (shape = (height, width, 3))
        :return: None
        """
        self.video.write(frame)

    def stopVideo(self):
        """
        Close the initialized video capture.
        Type help(Capture.initVideoWriter) for details.

        :return: None
        """
        if self.video is not None:
            self.video.release()
            print("Video written to:\n" + self.videoOutputDir)

    def getFrame(self, readNextFrame=True):
        """
        A method to be used inside of a while loop. Reads frames from a video
        or a live capture.

        :param readNextFrame: True by default. Use to refresh the capture's
                current frame without proceeding to the next
        :return: A numpy array containg the frame captured
                (shape = (height, width, 3))
        """
        if self.isRunning is False:
            self.stopCamera()
            return
        if readNextFrame is False:
            self.decrementFrame()
        if self.frameSkip > 0:
            if type(self.camSource) == str:
                current = self.camera.get(cv2.CAP_PROP_POS_FRAMES)
                self.camera.set(cv2.CAP_PROP_POS_FRAMES,
                                current + self.frameSkip)
        success, self.frame = self.camera.read()

        if success is False or self.frame is None:
            if type(self.camSource) == int:
                raise Exception("Failed to read from camera!")
            elif self.loopVideo == True:
                self.setFrame(0)
                while success is False or self.frame is None:
                    success, self.frame = self.camera.read()
            else:
                self.stopCamera()
                if not self.quitOnEnd:
                    return None  # it's a video. stop the loop
                else:
                    print("Quitting...")
                    quit()

        if type(self.camSource) == str:
            if self.frame.shape[0:2] != (self.height, self.width):
                self.frame = cv2.resize(self.frame, (self.width, self.height),
                                   interpolation=cv2.INTER_NEAREST)

            cv2.setTrackbarPos(self.trackbarName, self.windowName,
                               int(self.camera.get(
                                       cv2.CAP_PROP_POS_FRAMES)))

        if self.dimensions is not None:
            x0, y0, x1, y1 = self.dimensions
            self.frame = self.frame[y0:y1, x0:x1]

        return self.frame
