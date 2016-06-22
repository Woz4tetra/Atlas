import sys

import numpy as np
import pykalman
import pylab

sys.path.insert(0, '../')

from microcontroller.logger import Parser

parser = Parser("Mon Jun 13 21;14;38 2016", "Jun 13 2016")

t0 = 0.0
initial_long = 0.0
initial_lat = 0.0
for timestamp, name, values in parser:
    if name == 'gps':
        t0 = timestamp
        initial_long = -values['long']
        initial_lat = values['lat']
        break

filter = pykalman.KalmanFilter(
    observation_covariance=np.identity(4),
    transition_covariance=np.identity(4),
    initial_state_mean=np.array([initial_long, 0.0, initial_lat, 0.0]))

state_mean = np.array([initial_long, 0.0, initial_lat, 0.0])
covariance = np.identity(4)

prev_time = 0

x, y = [], []
kx, ky = [], []

for timestamp, name, values in parser:
    if name == 'gps':
        long = -values['long']
        lat = values['lat']

        x.append(long)
        y.append(lat)

        observation = np.array([long, 0.0, lat, 0.0])

        dt = timestamp - prev_time
        prev_time = timestamp

        # obs_matrix = np.array(
        #     [[1, dt, 0, 0],
        #      [0, 1, 0, 0],
        #      [0, 0, 1, dt],
        #      [0, 0, 0, 1]])
        #
        # trans_matrix = np.array(
        #     [[1, 0, 0, 0],
        #      [0, 1, 0, 0],
        #      [0, 0, 1, 0],
        #      [0, 0, 0, 1]])
        trans_matrix = np.identity(4)
        obs_matrix = np.array(
            [[1, dt, 0, 0],
             [0, 1, 0, 0],
             [0, 0, 1, dt],
             [0, 0, 0, 1]])

        state_mean, covariance = filter.filter_update(
            state_mean, covariance,
            observation=observation,
            transition_matrix=trans_matrix,
            observation_matrix=obs_matrix
        )

        kx.append(state_mean[0])
        ky.append(state_mean[2])

pylab.plot(x, y, '-', kx, ky, '--')
pylab.xlabel('X position')
pylab.ylabel('Y position')
pylab.title('Measurement of GPS path')
pylab.legend(('measured', 'kalman'))
pylab.show()
