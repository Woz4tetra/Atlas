"""
HELPER FUNCTIONS
----------------
These functions come from files: GNSS_LS_position_velocity.m, 
Initialize_LC_P_matric.m, and Initialize_NED_attitude.m 

some other helper functions are included: Skew_symmetric.m, 
Euler_to_CTM.m
"""

import numpy as np
from math import *

# GNSS_LS_position_velocity.m
# -----------------------------------------------

# Global Variables
c = 299792458 # Speed of light in m/s
omega_ie = 7.292115E-5  # Earth rotation rate in rad/s

def GNSS_LS_position_velocity(GNSS_measurements,no_GNSS_meas,predicted_r_ea_e,predicted_v_ea_e):
	"""
	Parameters
	---------- 
	GNSS_measurements: GNSS measurement data:
	   Column 1              Pseudo-range measurements (m)
	   Column 2              Pseudo-range rate measurements (m/s)
	   Columns 3-5           Satellite ECEF position (m)
	   Columns 6-8           Satellite ECEF velocity (m/s)

	no_GNSS_meas: Number of satellites for which measurements are supplied
	predicted_r_ea_e: prior predicted ECEF user position (m)
	predicted_v_ea_e: prior predicted ECEF user velocity (m/s)

	Outputs
	----------
	est_r_ea_e: estimated ECEF user position (m)
	est_v_ea_e: estimated ECEF user velocity (m/s)
	est_clock: estimated receiver clock offset (m) and drift (m/s)

	"""

	x_pred = np.matrix(np.zeros((4,1))) # Initialize x_pred matrix
	x_pred[0:3] = predicted_r_ea_e
	x_est = np.matrix(np.zeros((4,1))) # Initialize x_est matrix
	test_convergence = 1
	est_clock = np.matrix(np.zeros((2,1))) # Initialize est_clock matrix

	while test_convergence > 0.0001:
		
		pred_meas = np.matrix(np.zeros((no_GNSS_meas,1))) # Initialize pred_meas matrix for use in for loop
		H_matrix = np.matrix(np.zeros((no_GNSS_meas,4))) # Initialize H_matrix for use in for loop
		delta_r = np.matrix(np.zeros((3,1))) # Initialize delta_r for use in for loop

		for k in range(1,no_GNSS_meas+1):
			# predict approx. range
			delta_r = np.subtract(np.transpose(GNSS_measurements[k-1,2:5]),x_pred)
			approx_range = math.sqrt(np.dot(delta_r.T, delta_r))

			# Calculate frame rotation during signal transit time
			C_e_I = np.matrix('1,0,0;0,1,0;0,0,1') # Initialize matrix and change values in following lines
			C_e_I[0,1] = omega_ie * approx_range / c
			C_e_I[1,0] = -omega_ie * approx_range / c

			# Predict pseudo-range
			delta_r = np.subtract(np.cross(C_e_I, np.transpose(GNSS_measurements[k-1,2:5])),x_pred[0:3])
			range = math.sqrt(np.dot(delta_r.T, delta_r))
			pred_meas[k-1] = range + x_pred[3]

			# Predict line of sight and deploy in measurement matrix
			H_matrix[k-1,0:3] = - np.transpose(delta_r) / range
			H_matrix[k-1,3] = 1

		# Unweighted least-squares solution
		x_est = x_pred + np.cross(np.cross(np.inverse(np.cross(H_matrix[0:no_GNSS_meas,:].T, \
				H_matrix[0:no_GNSS_meas,:])),H_matrix[0:no_GNSS_meas,:].T), np.subtract(\
				GNSS_measurements[0:no_GNSS_meas,0], pred_meas[0:no_GNSS_meas]))

		# Test convergence
		test_convergence = math.sqrt(np.dot(np.subtract(x_est, x_pred).T, np.subtract(x_est, x_pred)))

		# Set predictions to estimate
		x_pred = x_est

	# Set outputs to estimates
	est_r_ea_e = np.matrix(np.zeros((4,1))) # Initialize est_r_ea_a matrix to output
	est_r_ea_e[0:3] = x_est[0,3]
	est_r_ea_e[3] = x_est[3]
	est_clock[0] = x_est[3]

	# VELOCITY AND CLOCK DRIFT
	omega_ie_skewed = Skew_symmetric([0,0,omega_ie]) #REMEMBER TO IMPLEMENT Skew_symmetric from Skew_symmetric.m

	# Setup predicted state
	x_pred[0,3] = predicted_v_ea_e
	x_pred[3] = 0
	test_convergence = 1

	while test_convergence > 0.0001:
		
		pred_meas = np.matrix(np.zeros((no_GNSS_meas,1))) # Initialize pred_meas matrix for use in for loop
		u_as_e = np.matrix(np.zeros((3,1))) # Initialize u_as_e matrix for use in for loop
		delta_r = np.matrix(np.zeros((3,1))) # Initialize delta_r matrix for use in for loop
		H_matrix = np.matrix(np.zeros((no_GNSS_meas,4))) # Initialize H_matrix for use in for loop

		for k in range(1,no_GNSS_meas+1):
			# predict approx. range
			delta_r = np.subtract(np.transpose(GNSS_measurements[k-1,2:5]),est_r_ea_e)
			approx_range = math.sqrt(np.dot(delta_r.T, delta_r))

			# Calculate frame rotation during signal transit time
			C_e_I = np.matrix('1,0,0;0,1,0;0,0,1') # Initialize matrix and change values in following lines
			C_e_I[0,1] = omega_ie * approx_range / c
			C_e_I[1,0] = -omega_ie * approx_range / c

			# Calculate range
			delta_r = np.subtract(np.cross(C_e_I, np.transpose(GNSS_measurements[k-1,2:5])),est_r_ea_e)
			range = math.sqrt(np.dot(delta_r.T, delta_r))

			# Calculate line of sight
			u_as_e = delta_r / range

			# Predict pseudo-range
			range_rate = u_as_e.T * (C_e_I * (GNSS_measurements[k-1,5:8].T + \
						 omega_ie_skewed * GNSS_measurements[k-1,2:5].T) - \
						 (x_pred[0,3] + omega_ie_skewed * est_r_ea_e))
			pred_meas[k-1] = range_rate + x_pred[3]

			# Predict line of sight and deploy in measurement matrix
			H_matrix[k-1, 0:3] = - u_as_e.T
			H_matrix[k-1, 3] = 1

		# Unweighted least-squares solution
		x_est = x_pred + np.inverse(H_matrix[0:no_GNSS_meas,:].T * H_matrix) * \
				H_matrix[0:no_GNSS_meas,:].T * (GNSS_measurements[0:no_GNSS_meas,1] \
				- pred_meas[0:no_GNSS_meas])

		# Test convergence
		test_convergence = math.sqrt((x_est - x_pred).T * (x_est - x.pred))

		# Set predictions to estimates for next iteration
		x_pred = x_est

	# Set outputs to estimates
	est_v_ea_e = np.matrix(np.zeros((4,1)))
	est_v_ea_e[0:3] = x_est[0,3]
	est_clock[1] = x_est[3]

	# RETURN THE OUTPUTS

	return (est_r_ea_e, est_v_ea_e, est_clock)

def Skew_symmetric(a):
	"""
	Calculates a skew-symmetric matrix

	Parameters
	---------- 
	a: 3-element vector

	Outputs
	----------
	A: 3x3 matrix
	"""

	A = np.matrix(np.zeros((3,3))) #Initialize
	A[0,1] = -a[2]
	A[0,2] = a[1]
	A[1,0] = a[2]
	A[1,2] = -a[0]
	A[2,0] = -a[1]
	A[2,1] = a[0]

	return A

