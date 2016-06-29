import numpy as np


class KalmanFilter:
    def __init__(self, initial_state, initial_probability,
                 observation_matrix_H, process_error_covariance_Q):
        self.state = initial_state  # current state estimate
        self.prob = initial_probability  # current covariance estimate
        self.H = observation_matrix_H
        self.Q = process_error_covariance_Q

    def update(self, control_vector, measurement, state_transition_A, control_B,
               measurement_error_covariance_R):
        A = state_transition_A
        B = control_B
        R = measurement_error_covariance_R

        # Prediction step. Where will I be next update?
        # predicted state estimate
        state_est = A.dot(self.state) + B.dot(control_vector)

        # predicted covariance estimate
        prob_est = A.dot(self.prob).dot(np.transpose(A)) + self.Q

        # observation step. Where do my sensors say I am?
        innovation = measurement - self.H.dot(state_est)
        innovation_covariance = self.H.dot(prob_est).dot(np.transpose(
            self.H)) + R

        # update step. Update state and kalman gain matrix
        kalman_gain = prob_est.dot(np.transpose(self.H).dot(np.linalg.inv(
            innovation_covariance)))
        self.state = state_est + kalman_gain.dot(innovation)
        size = self.prob.shape[0]
        self.prob = (np.eye(size) - kalman_gain.dot(self.H)).dot(prob_est)

        return self.state
