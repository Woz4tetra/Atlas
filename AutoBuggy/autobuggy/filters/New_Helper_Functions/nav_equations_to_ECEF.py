#nav equations to ecef temp

def Nav_equations_ECEF(tor_s, prev_r_eb_e, prev_v_eb_e, prev_C_b_e, f_ib_b, omega_ib_b):
    # this section is attitude update
    alpha_ie = omega_ie * tor_s
    C_Earth = np.matrix([[cos(alpha_ie), sin(alpha_ie), 0],
               [-sin(alpha_ie), cos(alpha_ie), 0],
               [0,0,1]])
    # calculate attitude increment, magnitude, and skew-symmetric matrix
    alpha_ib_b = omega_ib_b * tor_s
    a_b_sq = alpha_ib_b.T*alpha_ib_b
    mag_alpha = np.sqrt(a_b_sq)
    Alpha_ib_b = skew_symmetric(alpha_ib_b)

    #Obtain coordinate transformation matrix
    if mag_alpha > 1E-8:
        C_new_old = (((np.eye(3) + sin(mag_alpha) / mag_alpha)*(Alpha_ib_b))+
                     (((1-cos(mag_alpha)) / mag_alpha**2 * Alpha_ib_b)*( Alpha_ib_b)))
    else:
        C_new_old = np.eye(3) + Alpha_ib_b

    C_b_e = (C_Earth*((est_C_b_e)*(C_new_old)))

    if mag_alpha > 1E-8:
        ave_C_b_e = (old_C_b_e * (np.eye(3) + (1-cos(mag_alpha)) / mag_alpha**2
                                  * Alpha_ib_b + (1-sin(mag_alpha)/mag_alpha) / mag_alpha**2
                                  * Alpha_ib_b * Alpha_ib_b) -
                     0.5*skew_symmetric([0,0,alpha_ie]) * est_C_b_e)
    else:
        ave_C_b_e = est_C_b_e - 0.5 * skew_symmetric([0,0,alpha_ie])*(est_C_b_e)

    #Transform specific force to ECEF-frame resolving axes 
    f_ib_e = ave_C_b_e*(f_ib_b)
    # update velocity
    v_eb_e = est_v_eb_e + tor_s * (f_ib_e + Gravity_ECEF(est_r_eb_e) - 2 *
                                   skew_symmetric([0,0,omega_ie])*(est_v_eb_e))
    # update cartesian position
    r_eb_e = est_r_eb_e + (v_eb_e*(est_v_eb_e)) * 0.5 * tor_s
    
    return (r_eb_e, v_eb_e, C_b_e)