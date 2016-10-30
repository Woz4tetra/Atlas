"""
This file contains a bunch of helper functions
"""
import numpy as np
from math import *


#these are globals
R_0 = 6378137 #Equatorial radius in meters
e = 0.0818191908425 #Earth gravitational constant (m^3 s^-2)
omega_ie = 0.00007292115 #Earth rotation rate (rad/s)
mu = 3.986004418 * 10**14 
J_2 = 1.082627*10**(-3)

def Gravity_ECEF(pos_ECEF):
    """
    % Inputs:
    %   pos_ECEF  Cartesian position of body frame w.r.t. ECEF frame, resolved
    %           about ECEF-frame axes (m)
    % Outputs:
    %   g       Acceleration due to gravity (m/s^2)
    """
    #Calculate distance from center of the Earth
    mag_r = sqrt(pos_ECEF*(pos_ECEF.T))

    #If the input position is 0,0,0, produce a dummy output
    if mag_r == 0:
        return [0,0,0]

    #Calculates gravitational acceleration
    else:
        z_scale = 5 * (pos_ECEF[2]/mag_r)**2
        
        matrix = np.matrix([(1-z_scale) * pos_ECEF[0],
                 (1-z_scale) * pos_ECEF[1],
                 (3-z_scale)* pos_ECEF[2]])
        gamma = (-mu / mag_r**3 *
                 (pos_ECEF + 1.5 * J_2 * (R_0 / mag_r)**2 * matrix))
        #Add centripetal acceleration
        g = [gamma[0] + omega_ie**2 * pos_ECEF[0],
             gamma[1] + omega_ie**2 * pos_ECEF[1],
             gamma[2]]
        return np.array(g)
        

def ECEF_to_NED(pos, vel, C_b_e):
    """
HAS A SUPER STUPID FLOATING POINT ERROR
%   pos        Cartesian position of body frame w.r.t. ECEF frame, resolved
%                 along ECEF-frame axes (m)
%   vel       velocity of body frame w.r.t. ECEF frame, resolved along
%                 ECEF-frame axes (m/s)
%   C_b_e         body-to-ECEF-frame coordinate transformation matrix
%
% Outputs:
%   L_b           latitude (rad)
%   lambda_b      longitude (rad)
%   h_b           height (m)
%   v_eb_n        velocity of body frame w.r.t. ECEF frame, resolved along
%                 north, east, and down (m/s)
%   C_b_n         body-to-NED coordinate transformation matrix

"""
    lambda_b = atan2(pos[1],pos[0])

    k1 = sqrt(1 - e**2) * abs(pos[2])
    k2 = e**2 * R_0
    beta = sqrt(pos[0]**2 + pos[1]**2)
    E = (k1 - k2) / beta
    F = (k1 + k2) / beta

    P = (4/3) * (E*F + 1)
    Q = 2 * (E*E - F*F)
    D = P**3 + Q*Q
    V = (sqrt(D) - Q) ** (1.0/3.0) - ( 
        (sqrt(D) + Q) ** (1.0/3.0))
    
    G = (0.5 * (sqrt(E*E + V) + E))

    T = sqrt(G**2 + (F-V*G) / (2*G-E)) - G #This becomes 0 when it shouldn't, rip
    
    L_b = sign(pos[2]) * atan((1 - T*T) / (2*T*sqrt(1 - e*e)))

    h_b = ((beta - R_0 * T) * cos(L_b) + (pos[2] - sign(pos[2]) *
           R_0 * sqrt(1-e*e)) * sin(L_b))

    # Calculate ECEF to NED coordinate transformation matrix
    cos_lat = cos(L_b)
    sin_lat = sin(L_b)
    cos_long = cos(lambda_b)
    sin_long = sin(lambda_b)

    C_e_n = np.matrix([[-sin_lat * cos_long, -sin_lat * sin_long, cos_lat],
                      [-sin_long, cos_long, 0],
                      [-cos_lat*cos_long, -cos_lat*sin_long, -sin_lat]])

    v_eb_n = C_e_n * (vel)
    C_b_n = C_e_n * (C_b_e)
    return C_b_n

def sign(x):
    if x < 0 :
        return -1
    elif x == 0:
        return 0
    else:
        return 1