from matrix import Matrix, quaternion_from_attitude_matrix
from vector import Vector
from quaternion import Quaternion

class MEKF:

    def __init__(self, R_input, P_start, x_init, inertial_acc_mag, dt, std_dev_process):
        self.R = R_input
        self.P_prop = P_start
        self.P_pre = P_start
        self.first_estimate_done = False
        self.q_est = Quaternion(1,0,0,0)
        self.q_prop = Quaternion(1,0,0,0)
        self.q_ref = Quaternion(1,0,0,0)
        self.del_x = Vector(0,0,0,0,0,0)
        self.x_prop = x_init
        self.x_est = x_init
        self.x_ref = x_init
        self.dt_prop = dt
        self.std_dev_gyro = std_dev_process
        self.z_reference = inertial_acc_mag

    def reset(self):
        eps = self.q_est.epsilon()
        del_theta = Vector(self.del_x[0], self.del_x[1], self.del_x[2])

        self.q_ref = self.q_ref + 0.5 * eps * del_theta
        self.q_ref.normalize()

        # Bias update
        for i in range(3,6):
            self.x_ref[i] = self.x_ref[i] + self.del_x[i]
        
        self.del_x = Vector(0,0,0,0,0,0)

    def measurement_update(self, acc_measure, mag_measure, inertial_propagation = -1):
        if inertial_propagation == -1:
            g_inertial = Vector(self.inertial_acc_mag.matrix[0:3])
            b_inertial = Vector(self.inertial_acc_mag.matrix[3:6])
        b_meas = mag_measure.unit_vector
        g_meas = acc_measure.unit_vector
        ## Has to be completed
        if self.first_estimate_done:
            
            pass
        else:
            ## Applying TRIAD
            v1 = g_meas
            v2 = v1.skew() * b_meas
            v2.normalize()
            v3 = v1.skew() * v2

            V = Matrix.from_list([v1, v2, v3]).transpose()

            w1 = g_meas
            w2 = w1.skew() * b_meas
            w2.normalize()
            w3 = w1.skew() * w2

            W = Matrix.from_list([w1, w2, w3]).transpose()

            A_start = W * V.transpose()

            q_start = quaternion_from_attitude_matrix(A_start)
            self.q_est = q_start
            self.q_prop = q_start
            self.q_ref = q_start
            self.first_estimate_done = True
        
    def predict_state(self, omega_meas):

        bias = Vector(self.del_x[0], self.del_x[1], self.del_x[2])
        w = omega_meas - bias
        dt = self.dt_prop
        w_cross = w.skew()

        F = Matrix.zeros(6,6)

        I3 = Matrix.identity(3)

        for i in range(3):
            for j in range(3):
                F[i][j] = - w_cross[i][j]
                F[i][j + 3] = - I3[i][j]
        
        I6 = Matrix.identity(6)
        phi = I6 + F * dt

        self.x_prop = self.x_prop + F * self.del_x * dt
        self.del_x = phi * self.del_x

        sigma_gyro_bias = Matrix.diagonal(*(self.std_dev_gyro.matrix[0:3]))
        sigma_gyro_noise = Matrix.diagonal(*(self.std_dev_gyro.matrix[3:6]))
        sigma_gyro_bias = sigma_gyro_bias * sigma_gyro_bias
        sigma_gyro_noise = sigma_gyro_noise * sigma_gyro_noise 
        
        Q1 = sigma_gyro_bias * dt + (1/3 * sigma_gyro_noise * (dt ** 3))
        Q2 = - 0.5 * sigma_gyro_noise * (dt ** 2) 
        Q4 = sigma_gyro_noise * dt

        Q_k = Matrix(6,6)

        for i in range(3):
            for j in range(3):
                Q_k[i][j] = Q1[i][j]
                Q_k[3 + i][j] = Q2[i][j]
                Q_k[i][3 + j] = Q2[i][j]
                Q_k[3 + i][3 + j] = Q4[i][j]

        self.P_pre = phi * self.P_prop * phi.transpose()
        self.P_prop = self.P_pre

        eps = self.q_prop.epsilon()
        self.q_est = self.q_prop + 0.5 * eps * w * dt
        self.q_est.normalize()
        self.q_prop = self.q_est 