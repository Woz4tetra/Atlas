import atlasbuggy

robot = atlasbuggy.init(
    gps, imu, steering, brakes
)

# - or -

robot = atlasbuggy.init(
    RoboQuasar()  # subclass of LiveRobot or Simulator
)

while True:


# overwrite __del__