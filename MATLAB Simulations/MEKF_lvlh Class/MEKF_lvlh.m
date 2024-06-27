classdef MEKF_lvlh
    
    % Class implementation for a Multiplicative Extended Kalman Filter
    % 6 * 1 state vector: 
    %   [1:3] = axis angle representation of rotation from orbit frame to body frame; (angle_of_rotation * unit_vector_along_rotation_axis) 
    %   [4:6] = gyro biases
    % Measurement vector includes sun-vector and magnetic field vector in sensor frame
    % Estimates the quaternion of the body frame w.r.t. orbit frame
    % Makes use of angular velocity measurement, propagator inputs of position and velocity to make predictions
    % Measurement update using sun-vector and magnetic field vector

    properties

        % _prop suffix added to propagated variables, they are propagated between methods
        
        R % measurement covariance 
        first_estimate_done % A boolean indicating whether q_prop has been initialised with TRIAD or not. Initially False, set to True once TRIAD estimate calculated
        P_update % meaurement updated state cov
        P_pre % predicted state cov
        P_prop % propagated state cov
        q_prop % propagated quaternion of body frame w.r.t. orbit frame
        q_est % estimated quaternion of body frame w.r.t. orbit frame
        del_x % error/incremental state vector
        x_ref % reference state vector
        x_pred % predicted state
        x_prop % propagated state
        x_est % estimated state
        q_ref % reference quaternion of body frame w.r.t. orbit frame
        predicted_meas % measurement prediction from propagated state
        dt_prop % propagation time between state predictions
        std_dev_gyro; % 6-D vector, first three components are std-dev in gyro bias measurements, next three are std dev in the gyro noise measurements
    end

    methods

        function obj = MEKF_lvlh(R_input, P_start, x_init, dt, std_dev_process)

            % Constructor; initialising properties

            obj.R = R_input;
            obj.q_prop = [1; 0; 0; 0];
            obj.q_ref = [1; 0; 0; 0];
            obj.q_est = [1; 0; 0; 0];
            obj.del_x = zeros(6, 1);
            obj.P_pre = P_start;
            obj.P_update = P_start;
            obj.P_prop = P_start;
            obj.x_ref = x_init;
            obj.dt_prop = dt;
            obj.x_prop = x_init; 
            obj.std_dev_gyro = std_dev_process;
            obj.first_estimate_done = false;

        end

        function obj = reset(obj)

            % Resets the x vector after setting del_x vector to zero,
            % updates the reference quaternion, and the gyro bias estimates 

            eps = epsilon(obj.q_ref);
            del_theta = obj.del_x(1:3); % the incremental angular dispacement vector from the reference quaternion

            obj.q_ref = obj.q_ref + 0.5 * eps * del_theta; % updatingr reference quaternion
            obj.q_ref = obj.q_ref / norm(obj.q_ref); % normalising the quaternion;

            obj.x_ref(4:6) = obj.x_ref(4:6) + obj.del_x(4:6); % updating gyro bias values in reference state vector
            obj.del_x = zeros(6, 1); % reset del_x to zero
        end

        function obj = measurement_update(obj, zk, sun_prop_lvlh, mag_prop_lvlh)
            
            % zk is the measurement(6 * 1) in body frame
            % Taking the measurement vector z as sun vector followed by magnetic field vector
            
            if obj.first_estimate_done == true

                % calculating xk(+)
                
                Rk = obj.R;
                sun_prop_lvlh = sun_prop_lvlh/norm(sun_prop_lvlh);
                mag_prop_lvlh = mag_prop_lvlh/norm(mag_prop_lvlh);

                % The predicted measurements of unit solar vector & unit magnetic field vector based on current state estimate
                hxk = pred_measurement(obj.q_prop, sun_prop_lvlh, mag_prop_lvlh); 

                Hk = sensitivity_matrix(hxk);
                
                % Kalman Gain Calculation
                P_pro = obj.P_prop;
                S = Hk * P_pro * Hk' + Rk;
                K = (P_pro * Hk') / S; % Kalman Gain K = P*H' * [(H*P*H' + R)^-1]


                zk = [zk(1:3)/norm(zk(1:3)); zk(4:6)/norm(zk(4:6))];
                K_del_z = K * (zk - hxk);
                obj.del_x = obj.del_x + K_del_z; % delta_x(+) = delta_x(-) + K * (zk - hxk)
                
                % Measurement Updating the quaternion
                del_theta= K_del_z(1:3); % the incremental angular dispacement vector from the reference quaternion
                obj.q_est = obj.q_prop + 0.5 * epsilon(obj.q_prop) * del_theta;
                obj.q_est = obj.q_est/norm(obj.q_est);
                
                % State and State Covariance Update
                obj.x_prop = obj.x_ref + obj.del_x;
                obj.P_update = (eye(6) - K * Hk) * obj.P_prop; % P(+) = (I - K * H) * P(-)

                % Keeping the latest estimates in propagation
                obj.P_prop  = obj.P_update;
                obj.q_prop = obj.q_est;
               
              else 
                % Apply TRIAD to determine the initial attitude estimate

                b_meas_first = zk(4:6); % First measurement of magnetic field in body frame
                sun_meas_first = zk(1:3); % First measurement of solar vector in body frame
                
                b_prop_first = mag_prop_lvlh; % The magnetic field in orbit frame as received from the propagator
                sun_prop_first = sun_prop_lvlh; % The solar vector in orbit frame as received from the propagator
                
                % TRIAD using magnetic field as first vector and solar vector as the second vector

                % Triad of vectors in orbit frame 'V'
                v1 = b_prop_first;
                v1 = v1/norm(v1);

                v2 = cross(v1, sun_prop_first);
                v2 = v2/norm(v2);

                v3 = cross(v1, v2);
                V = [v1, v2, v3];
                
                % Triad of vectors in body frame 'W'
                w1 = b_meas_first;
                w1 = w1/norm(w1);

                w2 = cross(w1, sun_meas_first);
                w2 = w2/norm(w2);

                w3 = cross(w1, w2);
                W = [w1, w2, w3];
                
                % The initial attitude is given by the quaternion q_start,
                % and corresponding attitude matrix A_start
                A_start = W * V';
                q_start = quaternion_from_attitude(A_start);
                
                % Initialising the filter properties, with the attitude estimate calculated
                obj.q_prop = q_start;
                obj.q_ref = q_start;
                obj.q_est = q_start;

                % Setting this to true 
                % since the first estimate has been calculated
                obj.first_estimate_done = true;
                
            end
            
        end

        function obj = predict_state(obj, omega_meas, pos_vec, vel_vec)
            % Takes the angular velocity measurement, velocity vector and position vector from the propagator and carries out state prediction
            % Calculating xk(-)
            
            
            % Calculating angular velocity of rotation of orbit frame(lvlh
            % frame) w.r.t inertial frame, as measured in inertial frame

            % Calculating the unit vectors along velocity vector and
            % position vector
            vel_vec_unit_vec = vel_vec/norm(vel_vec);
            pos_vec_unit_vec = pos_vec/norm(pos_vec);

            % Direction of the angular velocity of orbit frame w.r.t.
            % inertial frame, as measured in inertial frame
            w_lvlh_unit_vec = skew(pos_vec_unit_vec) * vel_vec_unit_vec;
            w_lvlh_unit_vec = w_lvlh_unit_vec/norm(w_lvlh_unit_vec);
            
            % The angular velocity would equal
            % (velocity_perpendicular_to_position_vector)/modulus(position vector)
            vel_dot_pos = vel_vec(1) * pos_vec_unit_vec(1) + vel_vec(2) * pos_vec_unit_vec(2) + vel_vec(3) * pos_vec_unit_vec(3); 
            vel_perpendicular_to_pos_vec = vel_vec - ((vel_dot_pos) * pos_vec_unit_vec);

            % w_lvlh_hat is the angular velocity of orbit frame(lvlh
            % frame) w.r.t inertial frame, as measured in inertial frame
            w_lvlh_hat = w_lvlh_unit_vec * norm(vel_perpendicular_to_pos_vec)/norm(pos_vec);
            

            % Constructing the attitude matrix of orbit frame w.r.t.
            % inertial frame
            z_att = - pos_vec_unit_vec;
            y_att = skew(z_att) * vel_vec_unit_vec;
            y_att = y_att/norm(y_att);
            x_att = skew(y_att) * z_att;

            % A_lvlh is the attitude matrix of orbit frame w.r.t. inertial frame
            A_lvlh = [x_att, y_att, z_att]';

            % A_body_wrt_lvlh is the attitude matrix of body frame w.r.t. orbit frame
            A_body_wrt_lvlh = attitude_matrix(obj.q_prop);

            % w_body is the sensor reading minus bias, angular velocity of body measured in body frame
            w_body = omega_meas - obj.x_prop(4:6);  % subtracting the bias
            
            % w is the rate of change of attitude error angle del_theta,
            % the rate of change of (orbit frame to body frame quaternion) in
            % body frame **
            w = w_body - A_body_wrt_lvlh * A_lvlh * w_lvlh_hat;
            
            I3 = eye(3);
            o3 = zeros(3,3);
            

            % propagating state-vector and error in state
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

            % Propagating the quaternion of body frame w.r.t orbit frame
            obj.q_est = obj.q_prop + 0.5 * epsilon(obj.q_prop) * w * obj.dt_prop;
            obj.q_est = obj.q_est/norm(obj.q_est);

            obj.q_prop = obj.q_est;
        end

    end

end

function a = attitude_matrix(q)

    % Returns the attitude matrix 
    % corresponding to rotation represented by quaternion q

    % vector_in_frame_F = attitude_matrix(q) * vector_in_fram_I 
    % (q is the quaternion of frame F w.r.t. I)
    
    qw = q(1); 
    qx = q(2);
    qy = q(3);
    qz = q(4);
    
    a = [
        1 - 2*(qy^2 + qz^2), 2*(qx*qy + qz*qw), 2*(qx*qz - qy*qw);
        2*(qx*qy - qz*qw), 1 - 2*(qx^2 + qz^2), 2*(qy*qz + qx*qw);
        2*(qx*qz + qy*qw), 2*(qy*qz - qx*qw), 1 - 2*(qx^2 + qy^2)
    ];
    end 

function H = sensitivity_matrix(meas_pre)

    % takes in the predicted sun-vector and magnetic field vector to return H
    % H = partial derivative of h(x) w.r.t. x
    % x is the state, h(x) is the measurement function

    b_pred = meas_pre(4:6);
    sun_pred = meas_pre(1:3);

    sun_skew = skew(sun_pred);
    b_skew = skew(b_pred);

    o3 = zeros(3,3);

    H = [sun_skew, o3; b_skew, o3];
end


function a = epsilon(q)

    % a helper matrix for quaternion multiplication
    % for a small rotation of delta_theta(small angle * unit_vector_along_rotation_axis) vector 
    % q_new = q_old + 0.5 * epsilon(q_old) * del_theta

    a = [-q(2:4)';
        q(1)*eye(3) + skew(q(2:4))];
end

function a = skew(x)

    % returns the cross product matrix after taking in a 3 * 1 vector
    % a vector cross b vector = cross_pdt_matrix(a) * b

    a = [0 -x(3) x(2);
         x(3) 0 -x(1);
         -x(2) x(1) 0];
end

function q = quaternion_from_attitude(M)

    % Returns the quaternion corresponding to the same attitude matrix
    % passed in as argument

    % Ensuring M is a 3x3 matrix
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
        % Case 2: m11 is the largest diagonal element
        S = sqrt(1.0 + M(1,1) - M(2,2) - M(3,3)) * 2; % S = 4 * qx
        qw = (-M(3,2) + M(2,3)) / S;
        qx = 0.25 * S;
        qy = (M(1,2) + M(2,1)) / S;
        qz = (M(1,3) + M(3,1)) / S;

    elseif M(2,2) > M(3,3)
        % Case 3: m22 is the largest diagonal element
        S = sqrt(1.0 + M(2,2) - M(1,1) - M(3,3)) * 2; % S = 4 * qy
        qw = (-M(1,3) + M(3,1)) / S;
        qx = (M(1,2) + M(2,1)) / S;
        qy = 0.25 * S;
        qz = (M(2,3) + M(3,2)) / S;

    else
        % Case 4: m33 is the largest diagonal element
        S = sqrt(1.0 + M(3,3) - M(1,1) - M(2,2)) * 2; % S = 4 * qz
        qw = (-M(2,1) + M(1,2)) / S;
        qx = (M(1,3) + M(3,1)) / S;
        qy = (M(2,3) + M(3,2)) / S;
        qz = 0.25 * S;

    end

    % Returning the quaternion as a column vector
    q = [qw; qx; qy; qz];
end

function res = pred_measurement(q_temp, sun_prop_orbit, mag_prop_orbit)

    % Returns the expected body frame measurement of the unit-vector in
    % direction of magnetic field &
    % unit vector in direction of magnetic-field vector
    % according to the current state vector and propagated field values in orbit frame**

    % returns h(xk(-)); where h(x) is the a 6 * 1 vector:
                        % Components[1:3] = solar vector in body frame
                        % Components[4:6] = mag_field in orbit frame


    % Attitude matrix w.r.t. current quaternion estimate
    A = attitude_matrix(q_temp);

    b_rotated = A * mag_prop_orbit; % predicted magnetic field unit vector in body frame
    sun_vec_rotated = A * sun_prop_orbit; % predicted solar unit vector in body frame
    res = zeros(6,1);

    res(1:3) = sun_vec_rotated;
    res(4:6) = b_rotated;
end

