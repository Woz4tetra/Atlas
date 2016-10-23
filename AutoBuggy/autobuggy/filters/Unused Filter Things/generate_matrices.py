
#TODO: write or find skew_symmetric()
#TODO: write or find transpose()
#

import numpy as np

class Generate_Matrices(object):

    def __init__(self, lever_arm, ins_body_to_ECEF, Q_num_max):
        self.omega_ie = 7.29211* 10^(-5) # Earth Rotation in rad/s
        self.Omega_ie = skew_symmetric([0,0, self.omega.ie])
        self.c = 299792458 #speed of light
        self.R_0 = 6378137 # WGS84 Equatorial radius in meters
        self.e = 0.0818191908425 # WGS84 eccentricity
        self.lever_arm = lever_arm #lever_arm between imu and gps

        self.ins_body_to_ECEF = ins_body

        self.Q_num = Q_num_max
        self.Q_state_array = []
        self.Q.state_error_array = []

    def generate_Phi(self, tao, specific_force, gravity_model_value):
        phi = np.eye(15) # initialize to identity bc will change later and what\
        #we don't change is an identity matrix

        # PHI

        # [F11  03     03    03  C*tao]
        # [F21 F22    F23 C*tao  03]
        # [ 03 I3*tao  I3     03 03]
        # [ 03  03     03     I3 03]
        # [ 03  03     03     03 I3]

        F11  = np.eye(3) - self.Omega_ie * tao
        
        F21 = (-1) * skew_symmetric(self.ins_body_to_ECEF * specific_force) * tao

        F22 = np.eye(3)  - 2 * self.Omega_ie * tao

        F23 = (-1) * (2 * gravity_model_value) * transpose(self.R_0) * tao / (
            (self.R_0 * WTF))

        #Check F23 bc could be wrong. LIKE SERIOUSLY. CHECK THIS SHITTY AF THING


        #goes through and replaces each block as noted above
        maybe = """replace_block(phi, F11, 0, 0, 3)
        replace_block(phi, F21, 3, 0, 3)
        replace_block(phi, F22, 3, 3, 3)
        replace_block(phi, F23, 3, 6, 3)

        
        replace_block(phi, self.Omega_ie * tao, 0, 12, 3)
        replace_block(phi, self.Omega_ie * tao, 3, 6, 3)
        replace_block(phi, np.eye(3)*tao, 6, 3, 3)"""
        phi[0:3,0:3] = F11
        phi[3:6,0:3] = F21
        phi[3:6,3:6] = F22
        phi[3:6,6:9] = F23

        
        phi[0:3,12:15] = self.Omega_ie*tao
        phi[3:6,9:12]  = self.Omega_ie*tao
        phi[6:9,3:6]   = np.eye(3) * tao
        

        return phi

    def generate_H(self):
        """
H:
    [[Hr1  03 -I3 03  03]
     [Hv1 -I3  03 03 Hv5]
     """

        H = np.zeros((6,15))
        Hr1 = skew_symmetric(self.Omega_ie * self.lever_arm)
        Hv1 = #Need to look at code to get this thing its weird
        Hv5 = self.Omega_ie * skew_symmetric(self.lever_arm)

        maybe = """replace_block(H, Hr1, 0,0,3) #making this a strong bc want for 
        replace_block(H, Hv1, 3,0,3)            #reference. same below
        replace_block(H, Hv5, 3,12,3)
        replace_block(H, -1*np.eye(3), 0,6,3)
        replace_block(H, -1*np.eye(3), 3,3,3)"""

        H[0:3,0:3]   = Hr1
        H[3:6,0:3]   = Hv1
        H[3:6,12:15] = Hv5

        H[0:3,9:12] = -np.eye(3)
        H[3:6,3:6]  = -np.eye(3)
          
        return H

    

    def generate_R(self, sigma_x, sigma_y, sigma_z, tao):
        """
R:
    [[omega_x, 0, 0],
     [0, omega_y, 0],
     [0, 0, omega_z]]
     """
        R = np.array([[omega_x, 0, 0],
                      [0, omega_y, 0],
                      [0, 0, omega_z]])

        return R

    def generate_Q(self, phi, x_k_posteriori, x_k_priori,
                   P_k_posteriori, P_k_minus_posteriori):
        """
Q_k = (1/N) * SUM from k-N to k of
    [dz_j * dz_j^T + P_k_posteriori - phi*P_kminus_posteriori*phi^T]


    Therefore, must store arrays of dz and P so that can use them in the future.
    Must be of length N, which is self.Q_num


    Note: oldest ones are at end of array,
    with at every iteration the first is removed and
    the new one is added to the end
    """
        dx = x_k_posteriori - x_k_priori
        self.Q_state_array.append(dx)
        self.Q.state_error_array.append(P_k_posteriori)

        if len(self.Q_state_matrix >= self.Q_num):
            self.Q_state_array = self.Q_state_array[1:]
        if len(self.Q_state_error_matrix >= self.Q_num):
        self.Q_state_error_array = self.Q_state_error_array[1:]

        Q_k = np.zeros(3) #need to check this size
        for i in range(len(self.Q_state_array)):
            #note length of both arrays should be the same
            Q_k += (self.Q_state_array[i]*np.transpose(self.Q_state_array[i]) +
                     P_k_posteriori - phi * P_k_minus_posteriori * np.transpose(phi))

        Q_k /= self.Q_num
        return Q_k

            
    def generate_dz(self, gps_pos, gps_vel,
                    ins_pos, ins_vel, ins_att, gyro_angular_rate):
        """
dz_k = [[ r_eaG - r_eb - C_b*I_ba]
        [v_eaG - v_eb - C_b(skew_symmetric(w_ib * l_ba) + omega_ie * C-b * I_ba]

        So basically, the difference between them
        minus the lever arm and frame transform

        """
        dz = np.zeros((6,3)) #check this size
        upper = gps_pos - ins_pos - ins_att * self.lever_arm
        lower = gps_vel - ins_vel - ins_att*(skew_symmetric(gyro_angular_rate))+(
            Self.omega_ie * ins_att *self.lever_arm)

        dz[0:3,:] = upper
        dz[3:6,:] = lower
        return dz
        

def replace_block(A, block, i, j, size):
    """
Replaces a block of size size in A starting at index i j with block
"""
    for row in range(size):
        for col in range(size):
            A[i+row][j+col] = block[row][col]
            
                     
