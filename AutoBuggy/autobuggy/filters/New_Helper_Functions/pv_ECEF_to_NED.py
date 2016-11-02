#pv_ECEF_to_NED.py
import numpy as np
from math import *

def pv_ECEF_to_NED(pos, vel, C_b_e):
    """
Converts Cartesian  to curvilinear position and velocity
% resolving axes from ECEF to NED
% Software for use with "Principles of GNSS, Inertial, and Multisensor
% Integrated Navigation Systems," Second Edition.
%
% This function created 11/4/2012 by Paul Groves
%
% Inputs:
%   r_eb _e        Cartesian position of body frame w.r.t. ECEF frame, resolved
%                 along ECEF-frame axes (m)
%   v_eb _e        velocity of body frame w.r.t. ECEF frame, resolved along
%                 ECEF-frame axes (m/s)
%
% Outputs:
%   L_b           latitude (rad)
%   lambda_b      longitude (rad)
%   h_b           height (m)
%   v_eb _n        velocity of body frame w.r.t. ECEF frame, resolved along
%                 north, east, and down (m/s)


CHECK WHAT EFPQDV MEAN!!!
LITERALLY THE SAME AS ECEF_to_NED,
SUBTLE DIFFERENCE AT THE END, PROBABLY DUE TO THE PV, LOOK MORE INTO THIS

"""


earth_radius = 6378137 #Equatorial radius in meters
eccentricity = 0.0818191908425 #Earth gravitational constant (m^3 s^-2)
earth_rotation_rate = 0.00007292115 #Earth rotation rate (rad/s)
gravitational_constant = 3.986004418 * 10**14 
J_2 = 1.082627*10**(-3)

    lambda_b = atan2(pos[1],pos[0])

    k1 = sqrt(1 - e**2) * abs(pos[2])
    k2 = e**2 * earth_radius
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
    
    L_b = sign(pos[2]) * atan((1 - T*T) / (2*T*sqrt(1 - eccentricity**2)))

    h_b = ((beta - earth_radius * T) * cos(L_b) + (pos[2] - sign(pos[2]) *
           earth_radius * sqrt(1-eccentricity**2)) * sin(L_b))

    # Calculate ECEF to NED coordinate transformation matrix
    cos_lat = cos(L_b)
    sin_lat = sin(L_b)
    cos_long = cos(lambda_b)
    sin_long = sin(lambda_b)

    C_e_n = np.matrix([[-sin_lat * cos_long, -sin_lat * sin_long, cos_lat],
                      [-sin_long, cos_long, 0],
                      [-cos_lat*cos_long, -cos_lat*sin_long, -sin_lat]])

    v_eb_n = C_e_n * (vel)
    return v_eb_n 


def sign(x):
    if x < 0 :
        return -1
    elif x == 0:
        return 0
    else:
        return 1