"""
This file contains a bunch of helper functions
"""
import numpy as np
from math import *
import decimal

#these are globals
R_0 = 6378137
e = 0.0818191908425
omega_ie = 0.00007292115
mu = 3.986004418 * 10**14
J_2 = 1.082627*10**(-3)

def Gravity_ECEF(pos_ECEF):
    mag_r = sqrt(pos_ECEF.dot(pos_ECEF.T))

    if mag_r == 0:
        return [0,0,0]

    else:
        z_scale = 5 * (pos_ECEF[2]/mag_r)**2
        
        array = np.array([(1-z_scale) * pos_ECEF[0],
                 (1-z_scale) * pos_ECEF[1],
                 (1-z_scale)* pos_ECEF[2]])
        gamma = (-mu / mag_r**3 *
                 (pos_ECEF + 1.5 * J_2 * (R_0 / mag_r)**2 * array))
        g = [gamma[0] + omega_ie**2 * pos_ECEF[0],
             gamma[1] + omega_ie**2 * pos_ECEF[1],
             gamma[2]]
        return np.array(g)


def skew_symmetric(A):
    """
creates a 3v3 skew_symmetric matrix from a 1x3 matrix
"""
    return np.array([[    0, -A[2], A[2]],
                     [ A[2],     0,-A[0]],
                     [-A[1],  A[0],   0]])
        

def pv_ECEF_to_NED(r_eb_e, v_eb_e):
    lambda_b = atan2(r_eb_e[1], r_eb_e[0])

    k1 = sqrt(1-e**2) * abs(r_eb_e[2])
    k2 = e**2 * R_0    
    beta = sqrt((r_eb_e[0])**2 + (r_eb_e[1])**2)


    E = (k1 - k2) / beta
    F = (k1 + k2) / beta

    # From (C.31)
    P = 4/3 * (E*F + 1)

    # From (C.32)
    Q = 2 * (E**2 - F**2)

    # From (C.33)
    D = P**3 + Q**2

    # From (C.34)
    V = (sqrt(D) - Q)**(1/3) - (sqrt(D) + Q)**(1/3)

    # From (C.35)
    G = 0.5 * (sqrt(E**2 + V) + E)

    # From (C.36)
    T = sqrt(G**2 + (F - V * G) / (2 * G - E)) - G

    # From (C.37)
    L_b = sign(r_eb_e[2]) * atan((1 - T**2) / (2 * T * sqrt (1 - e**2)))

    # From (C.38)
    h_b = ((beta - R_0 * T) * cos(L_b) +
           (r_eb_e[2] - sign(r_eb_e[2]) * R_0 * np.sqrt(1 - e**2)) * sin (L_b))
      
    # Calculate ECEF to NED coordinate transformation matrix using (2.150)
    cos_lat = cos(L_b)
    sin_lat = sin(L_b)
    cos_long = cos(lambda_b)
    sin_long = sin(lambda_b)
    C_e_n = np.array([[-sin_lat * cos_long, -sin_lat * sin_long,  cos_lat],
                      [-sin_long,            cos_long,        0],
                      [-cos_lat * cos_long, -cos_lat * sin_long, -sin_lat]])
     
    # Transform velocity using (2.73)
    v_eb_n = C_e_n * v_eb_e

    return (L_b, lambda_b, h_b, v_eb_n)

def Euler_to_CTM(eul):
    sin_phi = sin(eul[0])
    cos_phi = cos(eul[0])
    sin_theta = sin(eul[1])
    cos_theta = cos(eul[1])
    sin_psi = sin(eul[2])
    cos_psi = cos(eul[2])

    C = np.zeros((3,3))
    C[0][0] = cos_theta * cos_psi
    C[0][1] = cos_theta * sin_psi
    C[0][2] = -sin_theta
    C[1][0] = -cos_phi * sin_psi + sin_phi * sin_theta * cos_psi
    C[1][1] = cos_phi * cos_psi + sin_phi * sin_theta * sin_psi
    C[1][2] = sin_phi * cos_theta
    C[2][0] = sin_phi * sin_psi + cos_phi * sin_theta * cos_psi
    C[2][1] = -sin_phi * cos_psi + cos_phi * sin_theta * sin_psi
    C[2][2] = cos_phi * cos_theta

    return C

def NED_to_ECEF(L_b, lambda_b, h_b, v_eb_n, C_b_n):
    R_E = R_0 / np.sqrt(1 - (e * sin(L_b))**2)

    #convert position
    cos_lat = cos(L_b)
    sin_lat = sin(L_b)
    cos_long = cos(lambda_b)
    sin_long = sin(lambda_b)

    r_eb_e = np.array([[(R_E + h_b) * cos_lat * cos_long],
                       [(R_E + h_b) * cos_lat * sin_long],
                       [((1 - e**2) * R_E + h_b) * sin_lat]])

    C_e_n = np.array([[-sin_lat * cos_long, -sin_lat * sin_long, cos_lat],
                      [-sin_long          ,            cos_long,       0],
                      [-cos_lat * cos_long, -cos_lat * sin_long,-sin_lat]])

    
    v_eb_e = np.transpose(C_e_n).dot(v_eb_n)

    C_b_e  = np.transpose(C_e_n).dot(C_b_n)

    return C_b_e

def Nav_equations_ECEF(tor_s, est_r_eb_e, est_v_eb_e, est_C_b_e, f_ib_b, omega_ib_b):
    alpha_ie = omega_ie * tor_s
    C_Earth = np.array([[cos(alpha_ie), sin(alpha_ie), 0],
               [-sin(alpha_ie), cos(alpha_ie), 0],
               [0,0,1]])
    alpha_ib_b = omega_ib_b * tor_s
    a_b_sq = np.dot(alpha_ib_b.T,alpha_ib_b)
    mag_alpha = np.sqrt(a_b_sq)
    Alpha_ib_b = skew_symmetric(alpha_ib_b)

    if mag_alpha > 1 * 10 **(-8):
        C_new_old = ((np.eye(3) + sin(mag_alpha) / mag_alpha).dot(Alpha_ib_b) +
                     ((1-cos(mag_alpha)) / mag_alpha**2 * Alpha_ib_b).dot( Alpha_ib_b))
    else:
        C_new_old = np.eye(3) + Alpha_ib_b

    C_b_e = C_Earth.dot(est_C_b_e).dot(C_new_old)

    if mag_alpha > 1 * 10**(-8):
        ave_C_b_e = (est_C_b_e * (np.eye(3) + (1-cos(mag_alpha)) / mag_alpha**2
                                  * Alpha_ib_b + (1-sin(mag_alpha)/mag_alpha) / mag_alpha**2
                                  * Alpha_ib_b * Alpha_ib_b) -
                     0.5*skew_symmetric([0,0,alpha_ie]) * est_C_b_e)
    else:
        ave_C_b_e = est_C_b_e - 0.5 * skew_symmetric([0,0,alpha_ie]).dot(est_C_b_e)


    f_ib_e = ave_C_b_e.dot(f_ib_b)

    v_eb_e = est_v_eb_e + tor_s * (f_ib_e + Gravity_ECEF(est_r_eb_e) - 2 *
                                   skew_symmetric([0,0,omega_ie]).dot(est_v_eb_e))

    r_eb_e = est_r_eb_e + (v_eb_e.dot(est_v_eb_e)) * 0.5 * tor_s
    
    return (r_eb_e, v_eb_e, C_b_e)

def adapt_noise_covariance(Phi, P_new, P, Q, n, corrections):
    #NEEDS TO BE DONE
    #ADAPT TO STORE THE THINGS NECESSARY
    length = len(corrections)
    C = np.zeros((15,15))
    if (length < n):
        n = length

    for i in range(n):
        C = C + np.dot(np.dot(corrections[len(corrections)-i-1],P),
                       (corrections[len(corrections)- i-1]).T)

    Q = C / n + P_new - np.dot(np.dot(Phi, P),Phi.T)
    Q = np.diag(np.diag(Q))
    return Q
    

    
    
    return 0

def ECEF_to_NED(pos, vel, C_b_e):
    """
HAS A SUPER STUPID FLOATING POINT ERROR
"""
    lambda_b = atan2(pos[1],pos[0])

    k1 = sqrt(1 - e**2) * abs(pos[2])
    k2 = e**2 * R_0
    beta = sqrt(pos[0]**2 + pos[1]**2)
    E = (k1 - k2) / beta
    F = (k1 + k2) / beta

    P = (4/3) * (E*F+1)
    Q = 2 * (E*E - F*F)
    D = P**3 + Q*Q
    V = (sqrt(D) - Q) ** (1.0/3.0) - ( 
        (sqrt(D) + Q) ** (1.0/3.0))
    
    G = (0.5 * (sqrt(E*E + V) + E))

    T = sqrt(G**2 + (F-V*G) / (2*G-E)) - G #This becomes 0 when it shouldn't
    
    L_b = sign(pos[2]) * atan((1-T*T) / (2*T*sqrt(1-e*e)))

    h_b = ((beta - R_0 * T) * cos(L_b) + (pos[2] - sign(pos[2]) *
           R_0 * sqrt(1-e*e)) * sin(L_b))

    cos_lat = cos(L_b)
    sin_lat = sin(L_b)
    cos_long = cos(lambda_b)
    sin_long = sin(lambda_b)

    C_e_n = np.array([[-sin_lat * cos_long, -sin_lat * sin_long, cos_lat],
                      [-sin_long, cos_long, 0],
                      [-cos_lat*cos_long, -cos_lat*sin_long, -sin_lat]])

    v_eb_n = C_e_n.dot(vel)
    C_b_n = C_e_n.dot(C_b_e)
    return C_b_n

def sign(x):
    if x < 0 :
        return -1
    elif x == 0:
        return 0
    else:
        return 1

    


                
    


                                     
