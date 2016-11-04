#NED_to_ECEF.py

import numpy as np
from math import *


"""Converts curvilinear to Cartesian position, velocity
% resolving axes from NED to ECEF and attitude from NED- to ECEF-referenced
%
% Software for use with "Principles of GNSS, Inertial, and Multisensor
% Integrated Navigation Systems," Second Edition.
%
% This function created 2/4/2012 by Paul Groves
%
% Inputs:
%   L_b           latitude (rad)
%   lambda_b      longitude (rad)
%   h_b           height (m)
%   v_eb _n        velocity of body frame w.r.t. ECEF frame, resolved along
%                 north, east, and down (m/s)
%   C_b _n         body-to-NED coordinate transformation matrix
%
% Outputs:
%   r_eb _e        Cartesian position of body frame w.r.t. ECEF frame, resolved
%                 along ECEF-frame axes (m)
%   v_eb _e        velocity of body frame w.r.t. ECEF frame, resolved along
%                 ECEF-frame axes (m/s)
%   C_b _e         body-to-ECEF-frame coordinate transformation matrix
"""
earth_radius = 6378137 #Equatorial radius in meters
eccentricity = 0.0818191908425 #Earth gravitational constant (m^3 s^-2)
earth_rotation_rate = 0.00007292115 #Earth rotation rate (rad/s)
gravitational_constant = 3.986004418 * 10**14 

def ned_to_ecef_position(latitude,longitude,altitude):
	#Calculate transverse radius of curvature
	R_E = earth_radius / sqrt(1-(eccentricity*sin(latitude))**2)

	#convert position
	cos_lat = np.cos(latitude)
	sin_lat = np.sin(latitude)
	cos_long = np.cos(longitude)
	sin_long = np.sin(longitude)
	r_eb_e = np.matrix([(R_E + altitude) * cos_lat * cos_long,
          		(R_E + altitude) * cos_lat * sin_long,
          		((1 - e^2) * R_E + altitude) * sin_lat])

	return r_eb_e

def ned_to_ecef_general(north,east,down):
	cos_lat = np.cos(north)
	sin_lat = np.sin(north)
	cos_long = np.cos(east)
	sin_long = np.sin(east)
	#Calculate ECEF to NED coordinate transformation matrix
	c_e_n = np.matrix([-sin_lat * cos_long, -sin_lat * sin_long,  cos_lat,
                       -sin_long,            cos_long,            0,
                       -cos_lat * cos_long, -cos_lat * sin_long, -sin_lat])

	#transform velocity
	v_eb_e = c_e_n.T * np.matrix([north, east, down])

	return v_eb_e