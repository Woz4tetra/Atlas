#ECI_to_ECEF.py
"""
converts position, velocity, and attitude from EcI- to
% EcEF-frame referenced and resolved
%
% Software for use with "Principles of GNSS, Inertial, and Multisensor
% Integrated Navigation Systems," Second Edition.
%
% This function created 2/4/2012 by Paul Groves
%
% Inputs:
%   t             time (s)
%   r_ib _i        cartesian position of body frame w.r.t. ECI frame, resolved
%                 along ECI-frame axes (m)
%   v_ib _i        velocity of body frame w.r.t. ECI frame, resolved along
%                 ECI-frame axes (m/s)
%   c_b _i         body-to-ECI-frame coordinate transformation matrix
%
% Outputs:
%   r_eb _e        cartesian position of body frame w.r.t. EcEF frame, resolved
%                 along ECEF-frame axes (m)
%   v_eb _e        velocity of body frame w.r.t. ECEF frame, resolved along
%                 EcEF-frame axes (m/s)
%   c_b _e         body-to-ECEF-frame coordinate transformation matrix
"""
import numpy as np
from math import *

earth_rotation_rate = 0.00007292115

#calculate ECI to ECEF coordinate transformation matrix
def ECI_to_ECEF(time,r_ib_i,v_ib_i,c_b_i):
	c_i_e = np.matrix([cos(earth_rotation_rate*time),
					  sin(earth_rotation_rate*time),0
					  -sin(earth_rotation_rate*time),
					  cos(earth_rotation_rate*time),0
					  0,			0,				1])
	#transform position 
	r_eb_e = c_i_e *r_ib_i

	#Transforms velocity
	v_eb_e = c_i_e * (v_ib_i -earth_rotation_rate*
					  np.matrix([-r_ib_i[1],r_ib_i[0],0]))

	#Transform attitude
	c_b_e = c_i_e * c_b_i

	return (r_eb_e, v_eb_e, c_b_e)