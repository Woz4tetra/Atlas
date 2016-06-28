import numpy as np

class KalmanFilter:
    def __init__(self, state_transition_A, process_error_covariance_Q,
                 measurement_error_covariance_R, initial_state, initial_probability):
        self.A = state_transition_A
        self.Q = process_error_covariance_Q
        self.R = measurement_error_covariance_R
        self.state = initial_state  # current state estimate
        self.prob = initial_probability  # current covariance estimate
        self.H = None
    
    def update(self, control, measurement, observation_matrix_H, control_B):
        self.H = observation_matrix_H
        self.B = control_B
    
        # Prediction step. Where will I be next update?
        # predicted state estimate
        state_est = self.A * self.state + self.B * control
        
        # predicted covariance estimate
        prob_est = self.A * self.prob * np.transpose(self.A) + self.Q
        
        # observation step. Where do my sensors say I am?
        innovation = measurement - self.H * state_est
        innovation_covariance = self.H * prob_est * np.transpose(self.H) + self.R
        
        # update step. Update state and kalman gain matrix
        kalman_gain = prob_est * np.transpose(self.H) * np.linalg.inv(innovation_covariance)
        self.state = state_est + kalman_gain * innovation
        size = self.prob.shape[0]
        self.prob = (np.eye(size) - kalman_gain * self.H) * prob_est
        
        return self.state

class BuggyFilter:
    def __init__(self, initial_lat, initial_long, initial_heading):
        state_transition = np.array(
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0]
        )
        
        process_error_covariance = np.eye(6)
        measurement_error_covariance = np.eye(6)
        
        initial_state = np.array(
            [initial_lat, initial_long, initial_heading, 0, 0, 0]
        )
        
        initial_probability = np.eye(6)
        
        self.filter = KalmanFilter()
    
    def update(self, gps_x_long, gps_y_lat, gps_course, gps_vx, gps_vy,
               enc_x_long, enc_y_lat, enc_course, enc_vx, enc_vy,
               imu_angular_vel):
        pass
