from matrix import Matrix
from vector import Vector
from quaternion import Quaternion, quaternion_from_attitude_matrix

def sensitivity_matrix(pred_meas):
    ## pred_meas is expected to contain the unit vectors along measured acceleration and measured magnetic field
    assert(isinstance(pred_meas, Vector) and pred_meas.shape == (3,1), 'acc_prop inner should be a 3 * 1 vector')

    g_pred = Vector(pred_meas[0], pred_meas[1], pred_meas[2])
    b_pred = Vector(pred_meas[3], pred_meas[4], pred_meas[5])

    g_skew = g_pred.cross_pdt_matrix
    b_skew = b_pred.cross_pdt_matrix
    
    o3 = Matrix.zeros(3,3)

    H = Matrix(6,6)

    for i in range(3):
        for j in range(3):
            H[i][j] = g_skew[i][j]
            H[3 + i][j] = b_skew[i][j]
            H[3 + i][3 + j] = o3[i][j]
            H[i][3 + j] = o3[i][j]
    return H

def pred_measurement(q_curr, acc_prop_iner, mag_prop_iner):
    assert(isinstance(q_curr, Quaternion), 'First argument passed to pred_measurement should be a Quaternion')
    A = q_curr.attitude_matrix()

    assert(isinstance(acc_prop_iner, Vector) and acc_prop_iner.shape == (3,1), 'acc_prop inner should be a 3 * 1 vector')
    assert(isinstance(mag_prop_iner, Vector) and mag_prop_iner.shape == (3,1), 'mag_prop inner should be a 3 * 1 vector')

    b_rotated = A * mag_prop_iner
    g_rotated = A * acc_prop_iner

    hxk = Vector(g_rotated[0], g_rotated[1], g_rotated[2], b_rotated[0], b_rotated[1], b_rotated[2])

    return hxk

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
            g_inertial = Vector(self.z_reference[0], self.z_reference[1], self.z_reference[2])
            b_inertial = Vector(self.z_reference[3], self.z_reference[4], self.z_reference[5])
        else:
            g_inertial = Vector(inertial_propagation[0], inertial_propagation[1], inertial_propagation[2])
            b_inertial = Vector(inertial_propagation[3], inertial_propagation[4], inertial_propagation[5])    
        
        b_meas = mag_measure.unit_vector
        g_meas = acc_measure.unit_vector
        ## Has to be completed
        if self.first_estimate_done:
            Rk = self.R

            hxk = pred_measurement(self.q_prop, g_inertial, b_inertial)

            H = sensitivity_matrix(hxk)

            P_pro = self.P_prop
            H_T = H.transpose()
            S = H * P_pro * H_T + Rk
            K = (P_pro * H_T) * S.inverse()

            zk = Vector(acc_measure[0], acc_measure[1], acc_measure[2], mag_measure[0], mag_measure[1], mag_measure[2])
            K_del_z = K * (zk - hxk)
            self.del_x = self.del_x + K_del_z

            del_theta = Vector(K_del_z[0], K_del_z[1], K_del_z[2])
            self.q_est = self.q_prop + 0.5 * self.q_prop.epsilon() * del_theta
            self.q_est.normalize()

            I6 = Matrix.identity(6)

            self.x_prop = self.x_ref + self.del_x
            self.P_prop = (I6 - K * H) * self.P_prop

            self.q_prop = self.q_est

        else:
            ## Applying TRIAD
            v1 = g_inertial
            v2 = v1.cross_pdt_matrix * b_inertial
            v2.normalize()
            v3 = v1.cross_pdt_matrix * v2

            V = Matrix.from_list([v1, v2, v3]).transpose()

            w1 = g_meas
            w2 = w1.cross_pdt_matrix * b_meas
            w2.normalize()
            w3 = w1.cross_pdt_matrix * w2

            W = Matrix.from_list([w1, w2, w3]).transpose()

            A_start = W * V.transpose()

            q_start = quaternion_from_attitude_matrix(A_start)
            self.q_est = q_start
            self.q_prop = q_start
            self.q_ref = q_start
            self.first_estimate_done = True
        
    def predict_state(self, omega_meas):

        bias = Vector(self.x_prop[0], self.x_prop[1], self.x_prop[2])
        w = omega_meas - bias
        dt = self.dt_prop
        w_cross = w.cross_pdt_matrix

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

        sigma_gyro_bias = Matrix.diagonal(self.std_dev_gyro[0], self.std_dev_gyro[1], self.std_dev_gyro[2])
        sigma_gyro_noise = Matrix.diagonal(self.std_dev_gyro[3], self.std_dev_gyro[4], self.std_dev_gyro[5])
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

        self.P_pre = phi * self.P_prop * phi.transpose() + Q_k
        self.P_prop = self.P_pre

        eps = self.q_prop.epsilon()
        self.q_est = self.q_prop + 0.5 * eps * w * dt
        self.q_est.normalize()
        self.q_prop = self.q_est 
    