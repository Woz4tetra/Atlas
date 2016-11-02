#Gravity_ECEF
import numpy as np
from math import *


#these are globals
earth_radius = 6378137 #Equatorial radius in meters
eccentricity = 0.0818191908425 #Earth gravitational constant (m^3 s^-2)
earth_rotation_rate = 0.00007292115 #Earth rotation rate (rad/s)
gravitational_constant = 3.986004418 * 10**14 
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
        gamma = (-gravitational_constant / mag_r**3 *
                 (pos_ECEF + 1.5 * J_2 * (earth_radius / mag_r)**2 * matrix))
        #Add centripetal acceleration
        g = [gamma[0] + earth_rotation_rate**2 * pos_ECEF[0],
             gamma[1] + earth_rotation_rate**2 * pos_ECEF[1],
             gamma[2]]
        return np.array(g)
        