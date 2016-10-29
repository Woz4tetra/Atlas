from pykalman import KalmanFilter as pykf
from kalman_filter import KalmanFilter as benkf
import numpy as np
import time


def print_state(state1,state2):
    print("a = %2.2f, v = %2.2f, r = %2.2f" %(state1[0] - state2[0],
                                                state1[1] - state1[1],
                                                state1[2] - state2[2]))

def test():
    print("hI")
    init_state = np.array([0]*20)
    init_covariance = np.eye(20)
    process_covariance = np.eye(20)
    benKF = benkf(init_state, init_covariance, process_covariance)
    pyKF  = pykf (initial_state_mean = init_state,
                  initial_state_covariance = init_covariance,
                  transition_covariance = process_covariance)

    data = [[1]*20]*20

    dt = 1
    H = np.eye(20)
    A = np.eye(20)

    R = np.eye(20)

    
    ben_state = benKF.update(None, data[0], None, R, A, H)
    py_state, py_cov  = pyKF.filter_update(init_state, init_covariance, data[0], A,
                                    observation_matrix = H,
                                    observation_covariance = R)

    data = data[1:]
    
    for measurement in data:
        
        t0 = time.time()
        ben_state        = benKF.update(None, measurement, None, R, A, H)
        t1 = time.time()
        py_state, py_cov = pyKF.filter_update(py_state, py_cov, measurement, A,
                                       observation_matrix = H,
                                       observation_covariance = R)
        t2 = time.time()
        print_state(ben_state,py_state)
        print("\nben_time = %f, py_time = %f" %(t1-t0, t2-t1))
test()
