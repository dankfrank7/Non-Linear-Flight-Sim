function X_dot = IterRate(X, U, AircraftData)
% AERO3560 Flight Mechanics 1 A3
% Author: Thomas Ryan
%
% function [X_dot] = IterRate(X, U, AircraftData)
% 
% Info:
%   This function iterates on angular rates and state rate functions until
%   angular rates alpha_dot and beta_dot converge to a acceptable tolerance
%
% Inputs:
%   X: State vector [13x1 Column vector] 
%       u   X(1)    Velocity component in x_b axis                  [m/s]    
%       v   X(2)    Velocity component in y_b axis                  [m/s]                   
%       w   X(3)    Velocity component in z_b axis                  [m/s] 
%       p   X(4)    Body rate component in x_b axis (roll)          [rad/s]
%       q   X(5)    Body rate component in y_b axis (pitch)         [rad/s]
%       r   X(6)    Body rate component in z_b axis (yaw)           [rad/s]
%       q0  X(7)    Quaternion component 1
%       q1  X(8)    Quaternion component 2
%       q2  X(9)    Quaternion component 3
%       q3  X(10)   Quaternion component 4
%       xe   X(11)  X location in earth ref frame                   [m]
%       ye   X(12)  Y location in earth ref frame                   [m]
%       ze   X(12)  Z location in earth ref frame                   [m]
%
%   U: Control Vector [4x1 Column Vector]
%       delta_T     U(1)    Change in Thrust                        [N]    
%       delta_e     U(2)    Elevator Deflection                     [rad]
%       delta_a     U(3)    Airleron Deflection                     [rad]
%       delta_r     U(4)    Rudder Deflection                       [rad]
%
%   AircraftData: Structure containing aircraft parameters
%
% Outputs:
%   
%   X_dot: State vector rate of change [13x1 Column vector] 
%       u_dot   X(1)    RoC Velocity component in x_b axis          [m/s^2]    
%       v_dot   X(2)    RoC Velocity component in y_b axis          [m/s^2] 
%       w_dot   X(3)    RoC Velocity component in z_b axis          [m/s^2] 
%       p_dot   X(4)    RoC Body rate component in x_b axis (roll)[rad/s^2]
%       q_dot   X(5)    RoC Body rate component in y_b axis (pitch)[rad/s^2]
%       r_dot   X(6)    RoC Body rate component in z_b axis (yaw) [rad/s^2]
%       q0_dot  X(7)    RoC Quaternion component 1                    [/s]
%       q1_dot  X(8)    RoC Quaternion component 2                    [/s]
%       q2_dot  X(9)    RoC Quaternion component 3                    [/s]
%       q3_dot  X(10)   RoC Quaternion component 4                    [/s]
%       xe_dot   X(11)  RoC X location in earth ref frame             [m/s]
%       ye_dot   X(12)  RoC Y location in earth ref frame             [m/s]
%       ze_dot   X(12)  RoC Z location in earth ref frame             [m/s]
%
% Correct?: CORRECT

    % Set up iteration loop
    tol = 1*10^-8;  % This was just taken arbitrarily? Is this OK?
    limit = 400;
    
    % Make an initial geuss of 0 rad/s angular rate
    alpha_dot = 0;
    beta_dot = 0;
    
    % Loop through iterations until alpha and beta converge
    for i = 1:limit

        % Calculate the state rates using the initial angular rate geuss
        X_dot = StateRates(X, U, AircraftData, alpha_dot, beta_dot);
        
        % Estimate the angular rates 
        [alpha_dot_new, beta_dot_new] = AngularRates(X, X_dot);
        
        % Calculate errors 
        err_alpha = abs((alpha_dot-alpha_dot_new));
        err_beta = abs((beta_dot-beta_dot_new));
        % Update angular rates for iteration 
        alpha_dot = alpha_dot_new;
        beta_dot = beta_dot_new;
        
        % Check if both errors are within tolerance
        if (err_alpha < tol) && (err_beta < tol)
            
            % If so break the for loop
            break
            
        % Check if at iteration limit
        elseif i == limit
            
            % Display a message
            fprintf('Iteration Limit Reached (%f) - RateIter.m\n',limit)
            
        end
    end    
return