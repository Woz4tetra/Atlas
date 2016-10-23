"""

This file is the kalman filter, mostly following the groves textbook
using the modifications of the thesis

NOTE: change all ** to **

"""

import numpy as np
from numpy import sqrt
import math
from helper_functions import *

class KF(object):

    def __init__(self, init_pos, init_vel, init_angle):
        # note have to make sure these are right initialiazations

        #currently initialized to ones from thesis
        deg_to_rad  = 0.01745329252
        rad_to_deg  = 1/deg_to_rad
        mug_to_mps2 = 9.80665*10**(-6)

        #this is LC_KF_CONFIG
        #need to be tuned / looked up
        self.gps_correction = [0, -0.1783567, -1.5190381]
        self.gyro_noise_PSD = (30 * deg_to_rad/60)**2
        self.accel_noise_PSD = (1000 * mug_to_mps2)**2
        self.accel_bias_PSD = 1 * 10**(-10)
        self.gyro_bias_PSD = 1 * 10**(-10)

        #used for bounding of covariances
        self.pos_SD_min = 0.0
        self.pos_SD_max = 100
        self.vel_SD_min = 5.1
        self.vel_SD_max = 100
        
        self.init_att_unc = deg_to_rad*1.5
        self.init_vel_unc = 1.62
        self.init_pos_unc = 0.1
        self.init_accel_bias_unc = 40*mug_to_mps2
        self.init_gyro_bias_unc =  353*deg_to_rad/3600
        
        self.gps_pos_stddev = 1.52
        self.gps_vel_stddev = 16
        
        self.n = 10
        
        #constants
        self.c = 29979248
        self.omega_ie = 7.292115 * 10**(-5) #Earth rotation rate rad/s
        self.Omega_ie = skew_symmetric([0,0,self.omega_ie])
        #NOTE Need to write skew_symmetric
        self.R_0 = 6378137 #WGS84 Equatorial Radius in m
        self.e = 0.0818191908425 #WGS84 eccentricity
        self.mu = 3.986004418 * 10**(14) #gravity constant Earth
        self.lever_arm = [0,0.3046, 0.9209209]

        #majorly used stuff
        self.GNSS_pos = [0,0,0]
        self.GNSS_vel = [0,0,0]
        self.est_vel_ecef = np.transpose(init_vel)
        self.est_pos_ecef = np.transpose(init_pos)
        self.est_IMU_bias = [0.079275626459682,
                             0.241364047070897,
                             0.734772061774872,
                             -0.029153756601168,
                             -0.000357971882895,
                             -0.015669929574187]
        self.Q = np.zeros((15,15))
        #CHANGING THIS CAUSES MASSIVE CHANGES BC WHERE ITS USED IS A LARGE COMPOUNDING ERROR
        #OTHERWISE THERE IS ONLY A SMALLISH COMPOUNDING ERROR
        self.x = [0]*15
        self.corrections = []

        #initialize INS from GPS
        self.est_lat_b, self.est_long_b, self.est_h_b, self.est_vel_eb_n =(
            pv_ECEF_to_NED(self.est_pos_ecef, self.est_vel_ecef))



        #initialize estimated attitude solution
        self.est_C_b_n = Euler_to_CTM(init_angle)
        self.est_C_b_ecef = NED_to_ECEF(self.est_lat_b, self.est_long_b,
                                                        self.est_h_b, self.est_vel_eb_n,
                                                        self.est_C_b_n)


        self.est_pos_ecef += (self.est_C_b_ecef.dot( self.lever_arm )+
                              self.est_C_b_ecef.dot( self.gps_correction))
        
        self.P = np.zeros((15,15))
        self.P[0:3,0:3]     = np.eye(3) * self.init_att_unc**2
        self.P[3:6,3:6]     = np.eye(3) * self.init_vel_unc**2
        self.P[6:9,6:9]     = np.eye(3) * self.init_pos_unc**2
        self.P[9:12,9:12]   = np.eye(3) * self.init_accel_bias_unc**2
        self.P[12:15,12:15] = np.eye(3) * self.init_gyro_bias_unc**2


        #for use with Q
        #count is how many updates have happened
        #num_Q is how many updates there are before Q is updated
        self.count = 0
        self.num_Q = 10

        #things for imu data
        self.imu_data = []
        #imu_data is of the form [accel, gyro], where both of these are 1x3 matrices

    def update_imu(self, data):
        """
        data is in the from [accel, gyro]

        where accel = [ax,ay,az]
        where gyro = [r,p,y]
        """
        self.imu_data.append(data)

    def update_gps(self, GNSS_pos_ecef, GNSS_vel_ecef, tor_s, GNSS_pdop):
        num_imu = len(self.imu_data)
        specific_force = [0,0,0]
        omega = [0,0,0]
        for i in range(num_imu):
            specific_force = np.add(specific_force,
                                    np.subtract(self.imu_data[i][0],
                                                self.est_IMU_bias[0:3]))
            
            omega = np.add(omega, np.subtract(self.imu_data[i][1],
                                              self.est_IMU_bias[3:6]))
        specific_force = np.divide(specific_force,num_imu)
        omega = np.divide(omega,num_imu)
        self.imu_data = []


        self.est_pos_ecef, self.est_vel_ecef, self.est_C_b_ecef = (
            Nav_equations_ECEF(tor_s, self.est_pos_ecef, self.est_vel_ecef,
                               self.est_C_b_ecef, specific_force, omega))

        #now update and run kf

        pos_variance = self.gps_pos_stddev**2 * GNSS_pdop**2
        vel_variance = self.gps_vel_stddev**2 * GNSS_pdop**2

        self.R = np.zeros((6,6))

        self.R[0:3,0:3] = np.diag(pos_variance * np.ones(3) * tor_s)
        self.R[3:6,3:6] = np.diag(pos_variance * np.ones(3) * tor_s)

        new_P, corrections = self.update_step(GNSS_pos_ecef, GNSS_vel_ecef, tor_s,
                                         specific_force, omega)

        self.corrections.append(corrections)

        self.Q = adapt_noise_covariance(self.Phi, new_P, self.P, self.Q,
                                        self.n, self.corrections)

        self.P = new_P

        self.est_C_b_n = ECEF_to_NED(self.est_pos_ecef,
                                               self.est_vel_ecef,
                                               self.est_C_b_ecef)
        
        
        
        
        
        
            


    def update_step(self, GNSS_pos_ecef, GNSS_vel_ecef, tor_s, meas_specific_force, meas_omega):

        #System Propogation Phase

        #1. Determine transition matrix
        F_21 = -skew_symmetric(self.est_C_b_ecef.dot( meas_specific_force))
        self.Phi = np.eye(15)
        self.Phi = np.eye(15);
        self.Phi[0:3,0:3] = self.Phi[0:3,0:3] - self.Omega_ie * tor_s
        self.Phi[0:3,12:15] = self.est_C_b_ecef * tor_s
        self.Phi[3:6,0:3] = tor_s * F_21
        self.Phi[3:6,3:6] = self.Phi[3:6,3:6] - 2 * self.Omega_ie * tor_s
        geocentric_radius = (self.R_0 / sqrt(1 - (self.e * sin(self.est_lat_b))**2)
                             * sqrt(cos(self.est_lat_b)**2 + (1 - (self.e)**2)**2
                                    * sin(self.est_lat_b)**2))
       
        self.Phi[3:6,6:9] = (-tor_s * 2 * Gravity_ECEF(self.est_pos_ecef) / geocentric_radius).dot(
            np.transpose(self.est_pos_ecef)) / sqrt(
                (np.transpose(self.est_pos_ecef).dot(self.est_pos_ecef)))


        self.Phi[3:6,9:12] = self.est_C_b_ecef * tor_s
        self.Phi[6:9,3:6] = np.eye(3) * tor_s

        #now decide whether or not to update Q matrix
        #2

        if self.count == self.num_Q:

            Srg = self.gyro_noise_PSD
            Sra = self.accel_noise_PSD
            Sbad = self.accel_bias_PSD
            Sbgd = self.gyro_bias_PSD

            Q = np.zeros((15,15))

            Q11 = np.eye(3) * (Srg * tor_s + Sbgd*tor_s**3/3)
            Q21 = (Srg*tor_s**2 / 2 + Sbgd*tor_s**4 / 4) * F_21
            Q31 = (Srg*tor_s**3 / 3 + Sbgd*tor_s**5 / 5) * F_21
            Q15 = Sbgd*tor_s**2*self.est_C_b_ecef / 2
            Q22 = (Sra*tor_s + Sbad*tor_s**3/3)*np.eye(3) + (Srg*tor_s**3/3 + Sbgd*tor_s**5/5)*(F_21.dot(F_21))
            Q32 = (Sra*tor_s**2/2 + Sbad*tor_s**4/4)*np.eye(3) + (Srg*tor_s**4/4 + Sbgd*tor_s**6/6)*(F_21.dot(F_21))
            Q24 = Sbad*tor_s**2*self.est_C_b_ecef / 2
            Q25 = Sbgd*tor_s**3*F_21*self.est_C_b_ecef / 3
            Q33 = (Sra*tor_s**3/3 + Sbad*tor_s**5/5)*np.eye(3) + (Srg*tor_s**5/5 + Sbgd*tor_s**7/7)*(F_21.dot(F_21))
            Q34 = Sbad*tor_s**3*self.est_C_b_ecef/3
            Q35 = Sbgd*tor_s**4*F_21*self.est_C_b_ecef/4
            Q44 = Sbad*tor_s*np.eye(3)
            Q55 = Sbgd*tor_s*np.eye(3)

            Q[0:3,0:3]   = Q11
            Q[0:3,3:6]   = np.transpose(Q21)
            Q[0:3,6:9]   = np.transpose(Q31)
            Q[0:3,9:12]  = np.zeros((3,3))
            Q[0:3,12:15] = Q15

            Q[3:6,0:3]   = Q21
            Q[3:6,3:6]   = Q22
            Q[3:6,6:9]   = np.transpose(Q32)
            Q[3:6,9:12]  = Q24
            Q[3:6,12:15] = Q25

            Q[6:9,0:3]   = Q31
            Q[6:9,3:6]   = Q32
            Q[6:9,6:9]   = Q33
            Q[6:9,9:12]  = Q34
            Q[6:9,12:15] = Q35

            Q[9:12,0:3]   = np.zeros((3,3))
            Q[9:12,3:6]   = np.transpose(Q24)
            Q[9:12,6:9]   = np.transpose(Q34)
            Q[9:12,9:12]  = Q44
            Q[9:12,12:15] = np.zeros(3)

            Q[12:15,0:3]   = np.transpose(Q15)
            Q[12:15,3:6]   = np.transpose(Q25)
            Q[12:15,6:9]   = np.transpose(Q35)
            Q[12:15,9:12]  = np.zeros(3)
            Q[12:15,12:15] = Q55

            self.Q = Q

        else:
            self.Q *= tor_s

        # here would be bounding minimum values of Q. TO ADD

        #3: Progogate state estimates noting that all
        #   states are zero due to closed loop correction
        
        self.x = np.array([0]*15)

        #4. Propogate state estimation error covariance
        self.P = self.Phi.dot(self.P).dot(np.transpose(self.Phi)).dot(self.Q)

        #Measurement Update Phase

        #5: Set up H
        self.H = np.zeros((6,15))
        self.H[0:3,0:3] = skew_symmetric(self.est_C_b_ecef.dot(self.lever_arm))
        self.H[3:6,0:3] = skew_symmetric(self.est_C_b_ecef.dot(np.cross(meas_omega, self.lever_arm))
                                         - (self.Omega_ie.dot(self.est_C_b_ecef)).dot(self.lever_arm))
        self.H[0:3,6:9]   = -np.eye(3)
        self.H[3:6,3:6]   = -np.eye(3)
        self.H[3:6,12:15] = self.est_C_b_ecef.dot(skew_symmetric(self.lever_arm))

        #6: Set up measurement noise covariance matrix assuming all components of
        #   GNSS position and velocity are independent and have equal variance

        #idk why this part was empty


        #7: calculate kalman gain

        step1 = np.dot(self.P, self.H.T)
        step2 = np.dot(self.H, self.P)
        step3 = np.dot(step2, self.H.T)
        step4 = step3 + self.R
        self.K = np.linalg.lstsq(step1.T, step4)[0]

        #8: formulate measurement innovations

        self.delta_z = np.array([0]*6)

        self.delta_z[0:3] = (GNSS_pos_ecef - self.est_pos_ecef
                               - self.est_C_b_ecef.dot(self.lever_arm)
                               - self.est_C_b_ecef.dot(self.gps_correction))

        self.delta_z[3:6] = (GNSS_vel_ecef - self.est_vel_ecef
                               - self.est_C_b_ecef.dot(np.cross(meas_omega,self.lever_arm))
                               + self.Omega_ie.dot(self.est_C_b_ecef).dot(self.lever_arm))
        

        #9: update state estimates  

        self.x = self.x + (self.K).dot(self.delta_z)
        corrections = self.x

        #10: update state estimation error covariance

        new_P = (np.eye(15) - self.K.dot(self.H)).dot(self.P)

        #here would be the bounds on P, not doing rn. TO DO

        #Closed Loop Corrections

        self.est_C_b_ecef = (np.eye(3) - skew_symmetric(self.x[0:3])).dot(self.est_C_b_ecef)
        self.est_vel_ecef -= self.x[3:6]
        self.est_pos_ecef -= self.x[6:9]

        self.est_IMU_bias -= self.x[9:15]
        return new_P, corrections
        
    def get_position(self):
        return self.est_pos_ecef
                      
        

            
                                          
        
