function [alpha_dot, beta_dot] = AngularRates(X,X_dot)
% AERO3560 Flight Mechanics 1 A3
% Author: Thomas Ryan
% 
% function [alpha_dot, beta_dot] = AngularRates(X,X_dot)
% 
% Info: 
%   Function takes state vector X and ROC state vector Xdor and estimates 
%   the angular rates of change for angle of attack (alpha) and sideslip
%   (beta) from u and v rates of change. This function will be used to
%   solve for alpha_dot and beta_dot iteratively using the following
%   method:
%      1) Get estimate of u_dot, v_dot and w_dot from previous time step
%      2) Estimate alpha_dot and beta_dot from u_dot, v_dot and w_dot
%      (this function)
%      3) Calculate X_dot from alpha_dot 
%      4) Iterate steps 1-3 until change is small 
%
% Inputs:
%   X: State vector [13x1 Column vector] 
%       u   X(1)    Velocity component in x_b axis                [m/s]    
%       v   X(2)    Velocity component in y_b axis                [m/s]                   
%       w   X(3)    Velocity component in z_b axis                [m/s] 
%       p   X(4)    Body rate component in x_b axis (roll)        [rad/s]
%       q   X(5)    Body rate component in y_b axis (pitch)       [rad/s]
%       r   X(6)    Body rate component in z_b axis (yaw)         [rad/s]
%       q0  X(7)    Quaternion component 1
%       q1  X(8)    Quaternion component 2
%       q2  X(9)    Quaternion component 3
%       q3  X(10)   Quaternion component 4
%       xe  X(11)   X location in earth ref frame                 [m]
%       ye  X(12)   Y location in earth ref frame                 [m]
%       ze  X(12)   Z location in earth ref frame                 [m]
%
%   X_dot: State vector rate of change [13x1 Column vector] 
%       u_dot   X(1)   RoC Velocity component in x_b axis         [m/s^2]    
%       v_dot   X(2)   RoC Velocity component in y_b axis         [m/s^2] 
%       w_dot   X(3)   RoC Velocity component in z_b axis         [m/s^2] 
%       p_dot   X(4)   RoC Body rate component in x_b axis (roll) [rad/s^2]
%       q_dot   X(5)   RoC Body rate component in y_b axis (pitch)[rad/s^2]
%       r_dot   X(6)   RoC Body rate component in z_b axis (yaw)  [rad/s^2]
%       q0_dot  X(7)   RoC Quaternion component 1                 [/s]
%       q1_dot  X(8)   RoC Quaternion component 2                 [/s]
%       q2_dot  X(9)   RoC Quaternion component 3                 [/s]
%       q3_dot  X(10)  RoC Quaternion component 4                 [/s]
%       xe_dot  X(11)  RoC X location in earth ref frame          [m/s]
%       ye_dot  X(12)  RoC Y location in earth ref frame          [m/s]
%       ze_dot  X(12)  RoC Z location in earth ref frame          [m/s]
%
% Outputs 
%   alpha_dot: rate of change of angle of attack                  [rad/s]
%   beta_dot:  rate of change of sideslip angle                   [rad/s]
%
% Required .m files: None
%
% Correct?: NOT TESTED

% Extract the aircraft state data
u = X(1);
v = X(2);
w = X(3);
u_dot = X_dot(1);
v_dot = X_dot(2);
w_dot = X_dot(3);

% Estimate the derivative of AoA using u_dot and v_dot
alpha_dot = (w_dot*u-w*u_dot)/u^2;

% Estimate the derivative of sideslip using u_dot and v_dot
beta_dot =  (v_dot*u-u_dot*v)/u^2;
return 