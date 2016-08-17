"""
A basic, but flexible Kalman filter.
Its implementation is based on Greg Czerniak's tutorial:
http://greg.czerniak.info/guides/kalman1/
"""
import numpy as np


class KalmanFilter:
    def __init__(self, initial_state, initial_probability,
                 process_error_covariance_q):
        self.state = initial_state  # current state estimate
        self.prob = initial_probability  # current covariance estimate
        self.Q = process_error_covariance_q

    def update(self, control_vector, measurement, control_b,
               measurement_error_covariance_r, state_transition_a,
               observation_matrix_h):
        self.H = observation_matrix_h  # converts state to a measurement
        self.A = state_transition_a

        # Prediction step. Where will I be next update?
        # predicted state estimate
        state_est = state_transition_a.dot(self.state) + \
                    control_b.dot(control_vector)

        # predicted covariance estimate
        prob_est = self.A.dot(self.prob).dot(np.transpose(self.A)) + self.Q

        # observation step. Where do my sensors say I am?
        innovation = measurement - self.H.dot(state_est)
        innovation_covariance = self.H.dot(prob_est).dot(np.transpose(
            self.H)) + measurement_error_covariance_r

        # update step. Update state and kalman gain matrix
        kalman_gain = prob_est.dot(np.transpose(self.H).dot(np.linalg.inv(
            innovation_covariance)))
        self.state = state_est + kalman_gain.dot(innovation)
        size = self.prob.shape[0]
        self.prob = (np.eye(size) - kalman_gain.dot(self.H)).dot(prob_est)

        return self.state
