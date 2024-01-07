function [V, alpha, beta] = AeroAngles(X)
% AERO3560 Flight Mechanics 1 A3
% Author: Thomas Ryan
% 
% function [V, alpha, beta] = AeroAngles(X)
% 
% Info: 
%   Estimates aero angles and total velocity from body axis vecotor
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
%       xe  X(11)  X location in earth ref frame                    [m]
%       ye  X(12)  Y location in earth ref frame                    [m]
%       ze  X(12)  Z location in earth ref frame                    [m]
%
% Outputs 
%   V: Flight velocity magnitude                                    [m/s]
%   alpha: Angle of attack                                          [rad]
%   beta: Sideslip angle                                            [rad]
%
% Required .m files: None
%
% Correct?: YES CORRECT

% Extract the aircraft state vector
u   = X(1);
v   = X(2);
w   = X(3);

% Calculate flight velocity magnitude
V = (u.^2+v.^2+w.^2)^0.5;

% Calculate aero angles
alpha = atan(w./u);
beta = asin(v./u);
return