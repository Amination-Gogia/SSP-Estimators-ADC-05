# MEKF LVLH Block for Simulink

This repository contains a MATLAB function to be used in a Simulink model for implementing a Multiplicative Extended Kalman Filter (MEKF) for attitude estimation in the Local Vertical Local Horizontal (LVLH) frame. The function estimates the quaternion of the body frame with respect to the orbit frame.

## Function: `mekf_lvlh_block`

### Description

This function performs attitude estimation using MEKF based on measurements from onboard sensors(sun-sensor and magnetometer). It uses the following inputs and provides the estimated quaternion as the output. 

### Usage Instructions

1. **Add the Function to Simulink**:
   - Add the [MEKF_lvlh.m](./MEKF_lvlh.m) file to the same directory as your simulink model, or add its directory to the MATLAB path.
   - Open your Simulink model.
   - Add a MATLAB Function block from the Simulink library.
   - Copy and paste the provided code into the MATLAB Function block. This function can also be copied from the [mekf_lvlh_block.m](./mekf_lvlh_block.m).
   
   ```matlab
   % Copy this to the MATLAB function block in Simulink to use the mekf_lvlh_block for attitude estimation
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
       persistent mekf

       if isempty(mekf)

           % Initialise an instance of the mekf_lvlh class
           % These values can be edited once we have done sensor modelling
           % And tested various values

           R_input = diag([0.005, 0.005, 0.005, 0.0007, 0.0007, 0.0007]); % Example values
           P_start = eye(6); % Example values
           x_init = [0;0;0;0.0;0.0;0.0]; %% The last three elements to be replaced by the GYRO BIAS VECTOR *******
           dt = 0.1; % Example time step
           std_dev_process = [0.01, 0.01, 0.01, 0.01, 0.01, 0.01]; % Example values

           mekf = MEKF_lvlh(R_input, P_start, x_init, dt, std_dev_process);
       end

       % Reset the MEKF object if required
       if norm(mekf.del_x (1:3)) > pi/18 % choosing when we want to reset the del_x and quaternion; this can be changed too
           mekf.reset();
       end

       % Measurement update
       zk = [sun_measurement_body; mag_measurement_body];
       mekf = measurement_update(mekf, zk, sun_propagated_lvlh, mag_propagated_lvlh);

       % Predict the state
       mekf = predict_state(mekf, omega_meas, pos_vec, vel_vec);

       q = mekf.q_est; % The estimate of the quaternion of body frame w.r.t orbit frame
   end
   ```

2. **Configure Inputs and Outputs**:

   - Ensure that the inputs to the function are provided as column vectors.
   - Connect the inputs and outputs appropriately in your Simulink model.

3. **Simulation**:
   - Run the simulation to estimate the quaternion of the body frame with respect to the orbit frame using the MEKF.

### Notes

- The function initializes an instance of the `mekf_lvlh` class the first time it is called and uses persistent variables to maintain state between function calls.
- The MEKF object is reset if the norm of the estimated error exceeds a certain threshold (set to π/18 radians in this example).
- The measurement and process noise covariance values (`R_input`, `P_start`, `std_dev_process`) and other parameters can be tuned based on sensor modeling and testing.
