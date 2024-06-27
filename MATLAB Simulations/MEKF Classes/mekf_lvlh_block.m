% Add this to the MATLAB function block in Simulink to use the mekf_lvlh_block for attitude estimation
% Ensure that MEKF_lvlh.m is in the same directory as your simulation 
% Or add the directory it is in to to the MATLAB path 

function q  = mekf_lvlh_block(omega_meas, sun_measurement_body, mag_measurement_body, sun_propagated_lvlh, mag_propagated_lvlh, pos_vec, vel_vec)

    
    
    % Inputs:(All inputs need to be as column vectors)
    %   omega_meas : Measured angular velocity in body frame
    %   sun_measurement_body: Unit solar vector measured in body frame 
    %   mag_measurement_body: Magnetic field vector measured in body frame
    %   sun_propagated_body: Unit solar vector in orbit(lvlh) frame, from propagator
    %   mag_propagated_body: Magnetic field vector in orbit(lvlh) frame, from propagator
    %   pos_vec is the position vector in inertial frame, from propagator
    %   vel_vec is the velocity vector in inertial frame, from propagator
    
    % Output: 
    %   q: quaternion of the body frame w.r.t. the orbit frame as a column vector

    % Persistent variable means that the function remembers this variable between function calls
    persistent mekf_new

    if isempty(mekf_new)

        % Initialise an instance of the mekf_new class
        % These values can be edited once we have done sensor modelling 
        % And tested various values

        R_input = diag([0.005, 0.005, 0.005, 0.0007, 0.0007, 0.0007]); % Example values
        P_start = eye(6); % Example values
        x_init = [0;0;0;0.0;0.0;0.0]; %% The last three elements to be replaced by the GYRO BIAS VECTOR *******
        dt = 0.1; % Example time step
        std_dev_process = [0.01, 0.01, 0.01, 0.01, 0.01, 0.01]; % Example values

        mekf_new = MEKF_lvlh(R_input, P_start, x_init, dt, std_dev_process);
    end


    % Reset the MEKF object if required
    if norm(mekf_new.del_x (1:3)) > pi/18 % choosing when we want to reset the del_x and quaternion; this can be changed too
        mekf_new.reset();
    end

    
    % Measurement update 
    zk = [sun_measurement_body; mag_measurement_body];
    mekf_new = measurement_update(mekf_new, zk, sun_propagated_lvlh, mag_propagated_lvlh);
    
    % Predict the state
    mekf_new = predict_state(mekf_new, omega_meas, pos_vec, vel_vec);
    

    q = mekf_new.q_est; % The estimate of the quaternion of body frame w.r.t orbit frame
end

