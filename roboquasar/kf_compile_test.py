
import numpy as np
from autobuggy.filters.kalman_filter import KalmanFilter

kf = KalmanFilter()
kf.filter_epoch(1, np.matrix([0, 0, 0]))