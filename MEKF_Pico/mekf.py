from matrix import Matrix
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

    def measurement_update(self, acc_measure, mag_measure):
        b = mag_measure.unit_vector
        g = acc_measure.unit_vector
        ## Has to be completed
        if self.first_estimate_done:
            
            pass
        
    def predict_state(self, omega_meas):

        bias = Vector(self.del_x[0], self.del_x[1], self.del_x[2])
        w = omega_meas - bias
        dt = self.dt_prop
        w_cross = w.skew()

        F = Matrix(6,6)

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
        ## Q calculation to be done

        
        eps = self.q_prop.epsilon()
        self.q_est = self.q_prop + 0.5 * eps * w * dt
        self.q_est.normalize()
        self.q_prop = self.q_est 