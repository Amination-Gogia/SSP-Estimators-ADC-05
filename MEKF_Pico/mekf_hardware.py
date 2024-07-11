from matrix import Matrix
from vector import Vector
from quaternion import Quaternion, quaternion_from_attitude_matrix

def sensitivity_matrix(pred_meas):

    ## Returns the sensitivity matrix 'H(xk(-))'
    ## pred_meas is expected to contain the unit vectors along predicted acceleration and magnetic field in body frame
    ## h(xk(-))
    assert(isinstance(pred_meas, Vector) and pred_meas.shape == (3,1), 'acc_prop inner should be a 3 * 1 vector')

    # Predicted unit vectors along gravity and mag field separated out
    g_pred = Vector(pred_meas[0], pred_meas[1], pred_meas[2])
    b_pred = Vector(pred_meas[3], pred_meas[4], pred_meas[5])

    g_skew = g_pred.cross_pdt_matrix
    b_skew = b_pred.cross_pdt_matrix
    
    o3 = Matrix.zeros(3,3)

    H = Matrix(6,6)
    ## H = [[g_skew  o3]
    ##      [b_swew  o3]]

    for i in range(3):
        for j in range(3):
            H[i][j] = g_skew[i][j]
            H[3 + i][j] = b_skew[i][j]
            H[3 + i][3 + j] = o3[i][j]
            H[i][3 + j] = o3[i][j]
    return H

def pred_measurement(q_curr, acc_prop_iner, mag_prop_iner):

    ## q_curr is the body frame quaternion
    ## acc_prop_iner and mag_prop_iner are the gravitational acceleration and mag. field in the reference frame
    ## For this demo, the reference frame is assumed to be at rest, and is supposed to be inertial
    ## If the frame is not inertial, we can pass in propagator data as arguments

    ## Returns h(xk(-)); the predicted measurements of acc. and mag. field in body frame

    assert(isinstance(q_curr, Quaternion), 'First argument passed to pred_measurement should be a Quaternion')

    ## Attitude Matrix of body frame w.r.t. reference frame
    A = q_curr.attitude_matrix()

    assert(isinstance(acc_prop_iner, Vector) and acc_prop_iner.shape == (3,1), 'acc_prop inner should be a 3 * 1 vector')
    assert(isinstance(mag_prop_iner, Vector) and mag_prop_iner.shape == (3,1), 'mag_prop inner should be a 3 * 1 vector')

    ## Transforming the reference frame vectors to the body frame
    b_rotated = mag_prop_iner.__rmul__(A)
    g_rotated = acc_prop_iner.__rmul__(A)

    ## Concatenating the predicted measurements into one predicted measurement vector
    hxk = Vector(g_rotated[0], g_rotated[1], g_rotated[2], b_rotated[0], b_rotated[1], b_rotated[2])

    return hxk

class MEKF:

    def __init__(self, R_input, P_start, x_init, inertial_acc_mag, dt, std_dev_process):

        ## Constructor

        self.R = R_input ## Measurement Covariance matrix, 6 * 6
        self.P_prop = P_start ## Propagated State Covariance, 6 * 6
        self.P_pre = P_start ## Predicted State Covariance, 6 * 6
        self.first_estimate_done = False ## A boolean to indicate if the first state estimate has been made
        self.q_est = Quaternion(1,0,0,0) ## Estimated Quaternion
        self.q_prop = Quaternion(1,0,0,0) ## Propagated Quaternion
        self.q_ref = Quaternion(1,0,0,0) ## Reference Quaternion(the 'global' attittude representation)
        self.del_x = Vector(0,0,0,0,0,0) ## The incremental, small change in state vector
        self.x_prop = x_init ## Propagated State
        self.x_est = x_init ## Estimated State
        self.x_ref = x_init ## The Reference State
        self.dt_prop = dt ## Propagation time between state predictions, (can be changed)

        ## A 6 * 1 vector, with the first three components as standard deviation in gyro measurement noise
        ## the next three components as standard deviation of rate of change in gyro_bias
        self.std_dev_gyro = std_dev_process 

        ## 6 * 1 vector : 
        # [gravity unit vector in reference frame
        #  mag. field unit vector in reference frame]
        self.z_reference = inertial_acc_mag


    def reset(self):

        ## Resets the del_x vector to zero,
        ## Updates the x_ref vector and the reference quaternion q_ref

        ## Update the reference quaternion
        eps = self.q_est.epsilon()
        del_theta = Vector(self.del_x[0], self.del_x[1], self.del_x[2])

        self.q_ref = self.q_ref + 0.5 * del_theta.__rmul__(eps)
        self.q_ref.normalize()

        ## Gyro bias update
        for i in range(3,6):
            self.x_ref[i] = self.x_ref[i] + self.del_x[i]
        
        ## Reset del_x to zero
        self.del_x = Vector(0,0,0,0,0,0)


    def measurement_update(self, acc_measure, mag_measure, inertial_propagation = -1):

        ## Measurement Update Step
        ## inertial_propagation is an optional argument, which we can pass data to, if our reference frame itself changes
        ## the inertial_propagation argument should be 6 * 1 vector:[acc UNIT VECTOR in ref. frame
        ##                                                           mag. field UNIT VECTOR in ref. frame]
        ## But if the reference frame is inertial, then no-need to pass in an argument, assumed that same reference vectors to be used
        
        ## acc_measure is the acceleration vector reading from the IMU
        ## mag_measure is the magnetic field vector reading from magnetometer
        ## (THESE TWO VECTORS NEED NOT BE UNIT VECTORS, THEY ARE NORMALIZED BELOW)

        ## Separating out the reference acceleration unit vector g_inertial
        ## ang reference mag. field unit vector b_inertial
        if inertial_propagation == -1:
            g_inertial = Vector(self.z_reference[0], self.z_reference[1], self.z_reference[2])
            b_inertial = Vector(self.z_reference[3], self.z_reference[4], self.z_reference[5])
        else:
            g_inertial = Vector(inertial_propagation[0], inertial_propagation[1], inertial_propagation[2])
            b_inertial = Vector(inertial_propagation[3], inertial_propagation[4], inertial_propagation[5])    
        
        ## Normalizing and storing the measurement vectors
        b_meas = mag_measure.unit_vector ## Normalized mag. field measurement in body frame
        g_meas = acc_measure.unit_vector ## Normalized acc. measurement in body frame
 
        if self.first_estimate_done:
            ## Normal MEKF measurement update

            Rk = self.R ## Measurement Noise Covariance Noise

            ## Predicted measurement according to the predicted quaternoin
            hxk = pred_measurement(self.q_prop, g_inertial, b_inertial) 

            ## Sensitivity matrix H(xk(-))
            H = sensitivity_matrix(hxk)

            ## Kalman Gain Calculation
            P_pro = self.P_prop ## The propagated state covariance
            H_T = H.transpose()
            S = H * P_pro * H_T + Rk
            K = (P_pro * H_T) * S.inverse() ## K = P*H' * (H*P*H' + R)^-1

            ## Calculation of correction in del_x
            zk = Vector(acc_measure[0], acc_measure[1], acc_measure[2], mag_measure[0], mag_measure[1], mag_measure[2])
            K_del_z = (zk - hxk).__rmul__(K) ## (zk is the clubbed measurement vector: acc_meas, mag_meas)
            self.del_x = self.del_x + K_del_z ## del_x(+) = del_x(-) + K(zk - h(xk))

            ## Correction in q_est
            ## del_theta: the small correction angle about which body frame is to be rotated
            ## del_theta measured in frame represented by q_prop
            del_theta = Vector(K_del_z[0], K_del_z[1], K_del_z[2]) 
            self.q_est = self.q_prop + 0.5 * del_theta.__rmul__(self.q_prop.epsilon())
            self.q_est.normalize()

            I6 = Matrix.identity(6)

            ## State and covariance update
            self.x_prop = self.x_ref + self.del_x
            self.P_prop = (I6 - K * H) * self.P_prop

            ## Ensuring the latest quaternion is in propagation
            self.q_prop = self.q_est

        else:
            ## Applying TRIAD to get an initial estimate of attitude

            ## First Triad of vectors in reference frame 'V'
            v1 = g_inertial
            #v2 = v1.cross_pdt_matrix * b_inertial
            #v2.normalize()
            v2 = b_inertial.__rmul__(v1.cross_pdt_matrix)
            v2.normalize()
            v3 = v2.__rmul__(v1.cross_pdt_matrix)
            
            #matrix has attribute transpose so conversion is needed
            V = Matrix.from_list([v1, v2, v3]).transpose()

            ## Secong Triad of vectors in body frame 'W'
            w1 = g_meas
            w2 = b_meas.__rmul__(w1.cross_pdt_matrix)
            w2.normalize()
            w3 = w2.__rmul__(w1.cross_pdt_matrix)
            
            W = Matrix.from_list([w1, w2, w3]).transpose()
            
            print(type(v1))
            print(type(v2))
            print(type(v3))
            print(type(w1))
            print(type(w2))
            print(type(w3))
            
            ## The attitude matrix at the start of filtering action, A_start
            A_start = W * V.transpose()

            ## Setting quaternion correspondign to A_start as the estimate in the filter equations
            q_start = quaternion_from_attitude_matrix(A_start)
            self.q_est = q_start
            self.q_prop = q_start
            self.q_ref = q_start

            self.first_estimate_done = True ## Since we have a first estimate now
        
    def predict_state(self, omega_meas):

        ## State Prediction Step
        ## Takes in the angular velocity readings in body frame(with bias): omega_meas to predict the state

        # Subtracting the bias
        bias = Vector(self.x_prop[0], self.x_prop[1], self.x_prop[2])
        w = omega_meas - bias

        dt = self.dt_prop ## Propagation interval
        w_cross = w.cross_pdt_matrix
        
        ## Carrying out state prediction
        ## State Transition Matrix phi = I6 + F * dt
        ## F = [ [- w_cross  - I3]
        ##       [   o3        o3]]
        F = Matrix.zeros(6,6)
        I3 = Matrix.identity(3)

        for i in range(3):
            for j in range(3):
                F[i][j] = - w_cross[i][j]
                F[i][j + 3] = - I3[i][j]
        
        I6 = Matrix.identity(6)
        phi = I6 + F * dt

        ## Predicting del_x
        self.x_prop = self.x_prop + self.del_x.__rmul__(F) * dt ## x_prop(t) = x_prop(t - 1) + F * del_x(t - 1)
        self.del_x = self.del_x.__rmul__(phi) ## del_x(t) = phi * del_x(t - 1)

        ## Process noise calculation
        ## Q_k calculation, this still has to be confirmed but seems to work
        ## Taken from Markley-Crassidis
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

        ## Predicting the state covariance
        self.P_pre = phi * self.P_prop * phi.transpose() + Q_k ## P_predicted = phi * P_prop * phi' + Q_k
        self.P_prop = self.P_pre ## keeping latest P in propagation

        ## Predicting the quaternion
        eps = self.q_prop.epsilon()
        self.q_est = self.q_prop + 0.5 * w.__rmul__(eps)* dt 
        self.q_est.normalize()
        self.q_prop = self.q_est ## keeping latest quaternion in propagation
    