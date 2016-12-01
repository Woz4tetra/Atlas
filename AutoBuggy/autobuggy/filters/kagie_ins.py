
import numpy as np

class INS(object):
    def __init__(self, init_pos, init_vel, init_att_trans):
        self.pos = init_pos
        self.vel = init_vel
        self.att_trans = init_att_trans

    def update(self, dt, accel, gyro, imu_bias):
        """
dt is length of time-step
accel is [x,y,z]
gyro  is [roll, pitch, yaw]
imu_bias is [x,y,z,roll,pitch,yaw]
"""
        accel -= imu_bias[0:3]
        gyro  -= imu_bias[3:6]

        angle_change = gyro * dt #assuming constant

        #now find mag_angle_change

        mag_change = unit_to_scalar(np.sqrt(angle_change.T * angle_change))

        skew_angle_change = skew_symmetric(angle_change)

        #now estimate attitude

        #not sure if right

        self.att_trans = self.att_trans * (np.matrix(np.eye(3)) + skew_angle_change)

        #end

        accel_ecef = self.att_trans * accel

        est_vel = self.vel + dt * accel_ecef

        #left it as est_vel here so that i can use it and self.vel at the same time
        #the next step

        self.pos = self.pos + (est_vel + self.vel) * 0.5 * dt

        self.vel = est_vel

        
        
