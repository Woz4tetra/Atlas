
import numpy as np
from autobuggy.filters.kalman_filter import KalmanFilter

kf = KalmanFilter()
kf.update_gps()