#euler_to_ctm.py

def euler_to_ctm(roll,pitch,yaw):
	"""
	Purpose
	----------
	Converts a set of Euler angles to the corresponding coordinate transformation matrix

	Parameters
	----------
	eul: Euler angles describing the rotation from beta to alpha in the order (roll, pitch, yaw)

	Output
	----------
	C: coordinate transformation matrix describing transformation from beta to alpha

	"""

	# precalculate sines and cosines of the Euler angles
	sin_phi = np.sin(roll)
	cos_phi = np.cos(roll)
	sin_theta = np.sin(pitch)
	cos_theta = np.cos(pitch)
	sin_psi = np.sin(yaw)
	cos_psi = np.cos(yaw)

	# calculate transformation matrix 
	C = np.matrix(np.zeros((3,3)))
	C[0,0] = cos_theta * cos_psi
	C[0,1] = cos_theta * sin_psi
	C[0,2] = -sin_theta
	C[1,0] = -cos_phi * sin_psi + sin_phi * sin_theta * cos_psi
	C[1,1] = cos_phi * cos_psi + sin_phi * sin_theta * sin_psi
	C[1,2] = sin_phi * cos_theta
	C[2,0] = sin_phi * sin_psi + cos_phi * sin_theta * cos_psi
	C[2,1] = -sin_phi * cos_psi + cos_phi * sin_theta * sin_psi
	C[2,2] = cos_phi * cos_theta

	return C