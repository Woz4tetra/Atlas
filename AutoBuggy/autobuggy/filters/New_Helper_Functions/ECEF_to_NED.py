#ECEF_to_NED.py
import numpy as np
from math import *


def ECEF_to_NED(x,y z):
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

#Change to
earth_radius = 6378137 #Equatorial radius in meters
eccentricity = 0.0818191908425 #Earth gravitational constant (m^3 s^-2)
earth_rotation_rate = 0.00007292115 #Earth rotation rate (rad/s)
gravitational_constant = 3.986004418 * 10**14 
J_2 = 1.082627*10**(-3)

    lambda_b = atan2(y,x)

    k1 = sqrt(1 - eccentricity**2) * abs(z)
    k2 = eccentricity**2 * earth_radius
    beta = sqrt(x**2 + y**2)
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
    return lambda_b,L_b,h_b


def sign(x):
    if x < 0 :
        return -1
    elif x == 0:
        return 0
    else:
        return 1