from vision.camera import Camera

class Logitech(Camera):
    def __init__(self, fps_size=None):
        resolutions = {
            8: (1920, 1080),
            12: (1440, 810),
            23: (960, 540),
            30: (480, 270),
            # 31: (240, 135),
            None: 30
        }

class ELP(Camera):
    def __init__(self, fps_size=None):
        resolutions = {
            10: (2048, 1536),
            12: (1600, 1200),
            30: (1280, 720),
            31: (640, 480),
            32: (320, 240),
            None: 30
        }

class ELP180(Camera):
    def __init__(self, fps_size=None):
        resolutions = {
            6: (1920, 1080),
            9: (1280, 720),
            21: (800, 600),
            30: (640, 480),
            31: (352, 288),
            32: (320, 240),
            None: 30
        }

class PS3eye(Camera):
    def __init__(self, fps_size=None):
        resolutions = {
            60: (640, 480),
            120: (320, 240),
            None: 60
        }