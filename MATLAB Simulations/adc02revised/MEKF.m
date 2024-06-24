classdef MEKF
    
    properties
        R % measurement covariance 
        first_estimate_done % A boolean
        P_update % meaurement updated state cov
        P_pre % predicted state cov
        P_prop % propagated state cov
        q_prop % propagated quaternion of body frame
        q_est % estimated quaternion
        del_x % error/incremental in state vector
        x_ref % reference state vector
        x_pred % predicted state
        x_prop % propagated state
        x_est % estimated state
        q_ref % reference quaternion
        predicted_meas % measurement prediction from propagated state
        dt_prop % propagation time between state predictions
        std_dev_gyro; % 6-D vector, first three components are std-dev in gyro bias measurements, next three are std dev in the gyro noise measurements
    end

    methods
        function obj = MEKF(R_input, P_start, x_init, dt, std_dev_process)
            obj.R = R_input;
            obj.q_prop = [0; 0; 0; 1];
            obj.q_ref = [0; 0; 0; 1];
            obj.q_est = [0; 0; 0; 1];
            obj.del_x = zeros(6, 1);
            obj.P_pre = P_start;
            obj.P_update = P_start;
            obj.P_prop = P_start;
            obj.x_ref = x_init;
            obj.dt_prop = dt;
            obj.x_prop = x_init; 
            %obj.predicted_meas = pred_measurement(obj.q_ref, [1; 0; 0], [0.1234; 0.4567; 0.7890]);
            obj.std_dev_gyro = std_dev_process;
            obj.first_estimate_done = false;
        end

        function obj = reset(obj)
            % Resets the x vector after setting dx vector to zero,
            % updates the reference quaternion 
            eps = epsilon(obj.q_ref);
            del_theta = obj.del_x(1:3); % the incremental angular dispacement vector from the reference quaternion

            obj.q_ref = obj.q_ref + 0.5 * eps * del_theta;
            obj.q_ref = obj.q_ref / norm(obj.q_ref); % normalising the quaternion;

            obj.x_ref(4:6) = obj.x_ref(4:6) + obj.del_x(4:6);
            obj.del_x = zeros(6, 1);
        end

        function obj = measurement_update(obj, zk, sun_prop_iner, mag_prop_iner) % zk is the measurement
            % Taking the measurement as sun vector followed by magnetic field vector
            if obj.first_estimate_done == true
                
            

            % calculating xk(+)
                Rk = obj.R;
                sun_prop_iner = sun_prop_iner/norm(sun_prop_iner);
                mag_prop_iner = mag_prop_iner/norm(mag_prop_iner);
                hxk = pred_measurement(obj.q_prop, sun_prop_iner, mag_prop_iner);
    
                Hk = sensitivity_matrix(hxk);
                P_pro = obj.P_prop;
                
                S = Hk * P_pro * Hk' + Rk;
                K = (P_pro * Hk') / S; % Kalman Gain
                
                zk = [zk(1:3)/norm(zk(1:3)); zk(4:6)/norm(zk(4:6))];
                obj.del_x = obj.del_x + K * (zk - hxk);
                
                obj.x_prop = obj.x_ref + obj.del_x;
                obj.P_update = (eye(6) - K * Hk) * obj.P_prop;
    
                obj.P_prop  = obj.P_update;
                
                K_del_z = K * (zk - hxk);
                del_theta= K_del_z(1:3);% the incremental angular dispacement vector from the reference quaternion
                obj.q_est = obj.q_prop + 0.5 * epsilon(obj.q_prop) * del_theta;
                obj.q_est = obj.q_est/norm(obj.q_est);
    
                obj.q_prop = obj.q_est;
            
             
                else 
                 
                b_meas_first = zk(4:6);
                sun_meas_first = zk(1:3);
                
                b_prop_first = mag_prop_iner;
                sun_prop_first = sun_prop_iner;
                
                v1 = b_prop_first;
                v1 = v1/norm(v1);
                v2 = cross(v1, sun_prop_first);
                v2 = v2/norm(v2);
                v3 = cross(v1, v2);
                V = [v1, v2, v3];

                w1 = b_meas_first;
                w1 = w1/norm(w1);
                w2 = cross(w1, sun_meas_first);
                w2 = w2/norm(w2);
                w3 = cross(w1, w2);
                W = [w1, w2, w3];
                
                A_start = W * V';
                q_start = quaternion_from_attitude(A_start);
                

                obj.q_prop = q_start;
                obj.q_ref = q_start;
                obj.q_est = q_start;
                obj.first_estimate_done = true;
                
            end
            
        end

        function obj = predict_state(obj, omega_meas)
            % Takes the angular velocity measurement and carries out state prediction
            % Calculating xk(-)
            w = omega_meas - obj.x_prop(4:6)  % subtracting the bias
            omega_meas = omega_meas
            temp= obj.x_prop(4:6)
            I3 = eye(3);
            o3 = zeros(3,3);
            F = [-skew(w), -I3; o3, o3];
            I6 = eye(6);
            phi = I6 + F * obj.dt_prop; % state transition matrix
            
            obj.x_prop = obj.x_prop + F * obj.dt_prop * obj.del_x;
            obj.del_x = phi * obj.del_x;
            

            % Qt calculation still has to be confirmed
            sigma_gyro_bias = diag([obj.std_dev_gyro(1)^2; obj.std_dev_gyro(2)^2; obj.std_dev_gyro(3)^2]);
            sigma_gyro_noise = diag([obj.std_dev_gyro(4)^2; obj.std_dev_gyro(5)^2; obj.std_dev_gyro(6)^2]);

            dt = obj.dt_prop;
            Qt = [sigma_gyro_bias * dt + 1/3 * sigma_gyro_noise * dt ^ 3, - sigma_gyro_noise * dt * dt/2; - sigma_gyro_noise * dt * dt/2, sigma_gyro_noise * dt ];
            obj.P_pre = phi * obj.P_pre * phi' + Qt; 

            % del_theta = obj.del_x(1:3); % the incremental angular dispacement vector from the reference quaternion
            obj.q_est = obj.q_prop + 0.5 * epsilon(obj.q_prop) * w * obj.dt_prop;
            obj.q_est = obj.q_est/norm(obj.q_est);

            

            obj.q_prop = obj.q_est;
        end
    end
end



function a = attitude_matrix(q)
    % Returns the attitude matrix of transformation from inertial to body frame

    q1 = q(1);
    q2 = q(2);
    q3 = q(3);
    q4 = q(4); % q4 is the scalar component

    a = [
        1 - 2*(q2^2 + q3^2), 2*(q1*q2 + q3*q4), 2*(q1*q3 - q2*q4);
        2*(q1*q2 - q3*q4), 1 - 2*(q1^2 + q3^2), 2*(q2*q3 + q1*q4);
        2*(q1*q3 + q2*q4), 2*(q2*q3 - q1*q4), 1 - 2*(q1^2 + q2^2)
    ];
end

function H = sensitivity_matrix(meas_pre)
% takes in the predicted sun-vector and magnetic field vector to return H
    b_pred = meas_pre(4:6);
    sun_pred = meas_pre(1:3);

    sun_skew = skew(sun_pred);
    b_skew = skew(b_pred);

    o3 = zeros(3,3);

    H = [sun_skew, o3; b_skew, o3];
end

function a = epsilon(q)
    % a helper matrix for quaternion multiplication
    a = [q(4)*eye(3) + skew(q(1:3));
         -q(1:3)'];
end

function a = skew(x)
    % returns the cross product matrix after taking in a 3 * 1 vector
    a = [0 -x(3) x(2);
         x(3) 0 -x(1);
         -x(2) x(1) 0];
end

function res = pred_measurement(q_temp, sun_prop_inertial, mag_prop_inertial)
    % Returns the body frame measurement of the sun-vector and the magnetic-field vector
    A = attitude_matrix(q_temp);
    b_rotated = A * mag_prop_inertial;
    sun_vec_rotated = A * sun_prop_inertial;
    res = zeros(6,1);
    res(1:3) = sun_vec_rotated;
    res(4:6) = b_rotated;
end

function q = quaternion_from_attitude(M)
    % Ensure M is a 3x3 matrix
    assert(all(size(M) == [3 3]), 'Input matrix must be 3x3');

    % Calculate trace of the matrix
    tr = M(1,1) + M(2,2) + M(3,3);

    if tr > 0
        % Case 1: Trace is positive
        S = sqrt(tr + 1.0) * 2; % S = 4 * qw
        qw = 0.25 * S;
        qx = (-M(3,2) + M(2,3)) / S;
        qy = (-M(1,3) + M(3,1)) / S;
        qz = (-M(2,1) + M(1,2)) / S;
    elseif M(1,1) > M(2,2) && M(1,1) > M(3,3)
        % Case 2: m00 is the largest diagonal element
        S = sqrt(1.0 + M(1,1) - M(2,2) - M(3,3)) * 2; % S = 4 * qx
        qw = (-M(3,2) + M(2,3)) / S;
        qx = 0.25 * S;
        qy = (M(1,2) + M(2,1)) / S;
        qz = (M(1,3) + M(3,1)) / S;
    elseif M(2,2) > M(3,3)
        % Case 3: m11 is the largest diagonal element
        S = sqrt(1.0 + M(2,2) - M(1,1) - M(3,3)) * 2; % S = 4 * qy
        qw = (-M(1,3) + M(3,1)) / S;
        qx = (M(1,2) + M(2,1)) / S;
        qy = 0.25 * S;
        qz = (M(2,3) + M(3,2)) / S;
    else
        % Case 4: m22 is the largest diagonal element
        S = sqrt(1.0 + M(3,3) - M(1,1) - M(2,2)) * 2; % S = 4 * qz
        qw = (-M(2,1) + M(1,2)) / S;
        qx = (M(1,3) + M(3,1)) / S;
        qy = (M(2,3) + M(3,2)) / S;
        qz = 0.25 * S;
    end

    % Return the quaternion as a column vector
    q = [qx; qy; qz; qw];
end

    % Combine into a quaternion with the scalar part as the last component
    