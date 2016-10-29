from generate_matrices import Generate_Matrices


class Filter(object):
    def __init__(self, lever_arm, ins_body_to_ECEF):
        self.gen = Generate_Matrices(lever_arm, ins_body_to_ins)
        self.phi_priori = np.eye(15)
        self.phi_posteriori = np.eye(15)
        self.Q_priori = np.zeros(6) #no idea if this is right size tbh
        self.Q_posteriori = np.zeros(6)
        self.R = np.eye(3)

    def update(self, specific_force, gravity_model_value, tao, sigma_x, sigma_y, sigma_z):
        """
1) compute Phi_k-1 and Q_k-1
2) (x_k)- = Phi_k-1 * (x_k-1)+
3) (P_k)- = Phi_k-1 * (P_k-1)+ * (Phi_k-1)^T + (Q_k-1)
4) compute H_k and R_k with measurement model
5) K_k = (P_k)- * (H_k)+ * (H_k * (P_k)- * (H_k)^T + R_k)^(-1)
6) formulate z_k
7) (x_k)+ = (x_k)- + K_k * (z_k - H_k * (x_k)-)
8) (P_k)+ = (I - K_k * H_k) * (P_l)- * (I - K_k * H_k)^T + K_k * R_k * (K_k)^T
9) Adapt Q_k according to system performance
"""
        #assuming (1) is done (ish)
        phi_k_minus = self.gen.generate_Phi(tao, specific_force, gravoty_model_value)

        #(2)
        x_k_priori = self.phi_k_minus_priori * x_k_minus_posteriori

        #(3)
        P_k_priori = self.phi_k_minus_priori * P_k_posteriori

        #(4)
        self.H = self.gen.generate_H()
        self.R = self.gen.generate_R(omega_x, omega_y, omega_z)

        #(5)
        K_k = P_k_priori * np.transpose(self.H) * np.inverse(
            self.H*P_k_priori*np.transpose(self.H)+R_k)

        #(6)
        #deal with this one later because involves ins
        z_k = self.gen.generate_dz()

        #(7)
        x_k_posteriori = x_k_priori + K_k(z_k - H_k * x_k_priori)

        #(8)
        I = ?? Identity?
        P_k_posteriori = (I - K_k * H_k) * P_k_priori * np.transpose(I - K_k * H_k) + (
            K_k * R_k * np.transpose(K_k))

        #(9)
        #will finish later
        #also need to figure out outputs etc and where they need to go
