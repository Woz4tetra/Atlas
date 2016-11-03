#Kinematics_ECEF.py
from math import *
import numpy as np
"""
%Kinematics_ECEF - calculates specific force and angular rate from input
%w.r.t and resolved along ECEF-frame axes
%
% Software for use with "Principles of GNSS, Inertial, and Multisensor
% Integrated Navigation Systems," Second Edition.
%
% This function created 1/4/2012 by Paul Groves
%
% Inputs:
%   tor_i         time interval between epochs (s)
%   C_b_e         body-to-ECEF-frame coordinate transformation matrix
%   old_C_b_e     previous body-to-ECEF-frame coordinate transformation matrix
%   v_eb_e        velocity of body frame w.r.t. ECEF frame, resolved along
%                 ECEF-frame axes (m/s)
%   old_v_eb_e    previous velocity of body frame w.r.t. ECEF frame, resolved
%                 along ECEF-frame axes (m/s)
%   r_eb_e        Cartesian position of body frame w.r.t. ECEF frame, resolved
%                 along ECEF-frame axes (m)
% Outputs:
%   f_ib_b        specific force of body frame w.r.t. ECEF frame, resolved
%                 along body-frame axes, averaged over time interval (m/s^2)
%   omega_ib_b    angular rate of body frame w.r.t. ECEF frame, resolved
%                 about body-frame axes, averaged over time interval (rad/s)
"""
earth_radius = 6378137 #Equatorial radius in meters
eccentricity = 0.0818191908425 #Earth gravitational constant (m^3 s^-2)
earth_rotation_rate = 0.00007292115 #Earth rotation rate (rad/s)
gravitational_constant = 3.986004418 * 10**14 
J_2 = 1.082627*10**(-3)

def kinematics_ECEF(delta_t,c_b_e, old_c_b_e, v_eb_e, old_v_eb_e,r_eb_e):
	if delta_t >0:
	#Determine the earth rotation over the update interval
	#alpha_ie = Omega_ie *
		rotation_angle = earth_rotation_rate *delta_t
		C_Earth = np.matrix(
			[cos(rotation_angle), sin(rotation_angle),0,
			-sin(rotation_angle), cos(rotation_angle),0,
						0		,			0		 ,1])

		#Obtain coordinate transformation matrix from the old attitude to the
		#new
		C_old_new = c_b_e.T *C_Earth * old_c_b_e

		#Calculate the approximate angular rate w.r.t an inertial frame. (what?)
		angular_rate_ib_b = np.matrix([0.5*(C_old_new[1,2] - C_old_new[2,1]),
									   0.5*(C_old_new[3,1] - C_old_new[1,3]),
									   0.5*(C_old_new[0,1]-C_old_new[1,0])])

		#Calculate and apply the scaling factor
		scaling_factor = acos(0.5 *(C_old_new[0,0]+C_old_new[1,1]+
			C_old_new[2,2]-1))

		if temp >2*10**-5:
			angular_rate_ib_b = angular_rate_ib_b*scaling_factor/sin(scaling_factor)
		
		#Calculate the average angular rate (output 1!)
		average_angular_rate = angular_rate_ib_b/delta_t

		#Calculate the specific force resolved about ECEF-fram axes (5.36)
		f_ib_e = (((v_eb_e - old_v_e_e)/delta_t) - gravity_ECEF(r_eb_e)
					+ 2*skew_symmetric([0,0,rotation_angle]) *old_v_eb_e)

		#Calculate the average body-to-ECEF frame coordinate transformation
		#matrix over the update interval using (5.84) and (5.85)
		mag_alpha = sqrt(angular_rate_ib_b.T * angular_rate_ib_b)
		skew_angular_rate = skew_symmetric(angular_rate_ib_b)
		if mag_alpha > 1*10**-8:
			average_c_b_e = old_c_b_e *(np.eye(3) + (1-cos(mag_alpha))/mag_alpha**2
				*skew_angular_rate + (1-sin (mag_alpha)/mag_alpha)/mag_alpha**2
				*skew_angular_rate*skew_angular_rate) - \
				0.5* skew_symmetric([0,0,rotation_angle]) * old_c_b_e
		else:
			average_c_b_e = old_c_b_e - 0.5*skew_symmetric([0,0]*old_c_b_e)

		f_ib_b = f_ib_e * np.linalg.inv(average_c_b_e) 
	else:
		average_angular_rate = [0,0,0]
		f_ib_b = [0,0,0]
	return average_angular_rate, f_ib_b
