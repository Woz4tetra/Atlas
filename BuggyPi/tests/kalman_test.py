import sys

import numpy
import pylab

sys.path.insert(0, '../')

from microcontroller.logger import Parser


# Implements a linear Kalman filter.
class KalmanFilterLinear:
    def __init__(self, _A, _B, _H, _x, _P, _Q, _R):
        self.A = _A  # State transition matrix.
        self.B = _B  # Control matrix.
        self.H = _H  # Observation matrix.
        self.current_state_estimate = _x  # Initial state estimate.
        self.current_prob_estimate = _P  # Initial covariance estimate.
        self.Q = _Q  # Estimated error in process.
        self.R = _R  # Estimated error in measurements.

    def GetCurrentState(self):
        return self.current_state_estimate

    def Step(self, control_vector, measurement_vector):
        # ---------------------------Prediction step-----------------------------
        predicted_state_estimate = self.A * self.current_state_estimate + self.B * control_vector
        predicted_prob_estimate = (
                                      self.A * self.current_prob_estimate) * numpy.transpose(
            self.A) + self.Q
        # --------------------------Observation step-----------------------------
        innovation = measurement_vector - self.H * predicted_state_estimate
        innovation_covariance = self.H * predicted_prob_estimate * numpy.transpose(
            self.H) + self.R
        # -----------------------------Update step-------------------------------
        kalman_gain = predicted_prob_estimate * numpy.transpose(
            self.H) * numpy.linalg.inv(innovation_covariance)
        self.current_state_estimate = predicted_state_estimate + kalman_gain * innovation
        # We need the size of the matrix so we can make an identity matrix.
        size = self.current_prob_estimate.shape[0]
        # eye(n) = nxn identity matrix.
        self.current_prob_estimate = (numpy.eye(
            size) - kalman_gain * self.H) * predicted_prob_estimate


# =============================REAL PROGRAM START================================
# Let's go over the physics behind the cannon shot, just to make sure it's
# correct:
# sin(45)*100 = 70.710 and cos(45)*100 = 70.710
# vf = vo + at
# 0 = 70.710 + (-9.81)t
# t = 70.710/9.81 = 7.208 seconds for half
# 14.416 seconds for full journey
# distance = 70.710 m/s * 14.416 sec = 1019.36796 m

dt = 1.0  # How many seconds should elapse per iteration?

# These are arrays to store the data points we want to plot at the end.
x = []
y = []
nx = []
ny = []
kx = []
ky = []

# Let's make a cannon simulation.

speedX = 0  # muzzle_velocity * math.cos(angle * math.pi / 180)
speedY = 0  # muzzle_velocity * math.sin(angle * math.pi / 180)

# This is the state transition vector, which represents part of the kinematics.
# 1, ts, 0,  0  =>  x(n+1) = x(n) + vx(n)
# 0,  1, 0,  0  => vx(n+1) =        vx(n)
# 0,  0, 1, ts  =>  y(n+1) =              y(n) + vy(n)
# 0,  0, 0,  1  => vy(n+1) =                     vy(n)
# Remember, acceleration gets added to these at the control vector.
state_transition = numpy.matrix(
    [[1, dt, 0, 0],
     [0, 1, 0, 0],
     [0, 0, 1, dt],
     [0, 0, 0, 1]])

control_matrix = numpy.matrix(
    [[1, 0, 0, 0],
     [0, 1, 0, 0],
     [0, 0, 1, 0],
     [0, 0, 0, 1]])
# The control vector, which adds acceleration to the kinematic equations.
# 0          =>  x(n+1) =  x(n+1)
# 0          => vx(n+1) = vx(n+1)
# -9.81*ts^2 =>  y(n+1) =  y(n+1) + 0.5*-9.81*ts^2
# -9.81*ts   => vy(n+1) = vy(n+1) + -9.81*ts
control_vector = numpy.matrix(
    [[0],
     [0],
     [0],
     [0]])

# After state transition and control, here are the equations:
#  x(n+1) = x(n) + vx(n)
# vx(n+1) = vx(n)
#  y(n+1) = y(n) + vy(n) - 0.5*9.81*ts^2
# vy(n+1) = vy(n) + -9.81*ts
# Which, if you recall, are the equations of motion for a parabola.  Perfect.

# Observation matrix is the identity matrix, since we can get direct
# measurements of all values in our example.
observation_matrix = numpy.eye(4)

# This is our guess of the initial state.  I intentionally set the Y value
# wrong to illustrate how fast the Kalman filter will pick up on that.
initial_state = numpy.matrix([[-71.42092895507812],
                              [speedX],
                              [42.4273567199707],
                              [speedY]])

initial_probability = numpy.eye(4)

process_covariance = numpy.zeros(4)
measurement_covariance = numpy.eye(4) * 0.2

kf = KalmanFilterLinear(state_transition, control_matrix, observation_matrix,
                        initial_state, initial_probability, process_covariance,
                        measurement_covariance)

# Iterate through the simulation.
parser = Parser("Mon Jun 13 21;14;38 2016", "Jun 13 2016")

prev_time = 0

for timestamp, name, values in parser:
    if name == 'gps':
        newestX = -values['long']
        newestY = values['lat']
        nx.append(newestX)
        ny.append(newestY)
        # Iterate the cannon simulation to the next timeslice.
        kx.append(kf.GetCurrentState()[0, 0])
        ky.append(kf.GetCurrentState()[2, 0])
        kf.Step(control_vector, numpy.matrix(
            [[newestX], [0], [newestY], [0]]))

# Plot all the results we got.
pylab.plot(x, y, '-', nx, ny, ':', kx, ky, '--')
pylab.xlabel('X position')
pylab.ylabel('Y position')
pylab.title('Measurement of GPS path')
pylab.legend(('true', 'measured', 'kalman'))
pylab.show()
