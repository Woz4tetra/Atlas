from filter_framework import Filter
from ins import INS


class KF_system(object):
    """
This is the very high level system that keeps track of communication
between the ins and the kf, updating both when necessary.

It keeps both the ins and the kf as separate objects.

The ins is updated when the imu inputs arrive, and
the kf is updated when the gps inputs arrive.

The kf takes in the current values of the ins,
then the output of the kf is used to correct the ins.

Finally, the ins value is returned.
"""
    def __init__(self, lever_arm, timestamp, num_inputs_ins, ins_body_to_ECEF):
        self.ins = INS(timestamp, num_inputs_ins)
        self.ins_time = timestamp

        self.kf  = Filter(lever_arm, ins_body_to_ECEF)
        self.kf_time = timestamp

    def update_imu(self, ax, ay, yaw, timestamp):
        self.ins.update(ax,ay,yaw,timestamp)
        self.ins_time = timestamp
        return self.ins.state

    def update_gps(self, specific_force, gravity_model_value, timestamp,
                   sigma_x, sigma_y, sigma_z):
        corrections = self.kf.update(specific_force, gravity_model_value,
                       timestamp - self.kf_time, sgma_x, sigma_y, sigma_z)
        self.kf_time = timestamp
        self.ins.correct(corrections)
        return self.ins.state
    
