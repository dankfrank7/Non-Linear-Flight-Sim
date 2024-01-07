% AERO3560 Flight Mechanics 1 A3
% Author: Jasraj Chouhan 
%
% function [FB_x, FB_y, FB_z] = Gravity(X,AircraftData)
%
% Info:
%   Function takes state vector X and aircraft data, to determine the
%   gravitional force felt by the aircraft. The function converts to the
%   body frame afterwards
%
% Inputs:
%   X: State vector [13x1 Column vector]
%   	u   X(1)	Velocity component in x_b axis              	[m/s]    
%   	v   X(2)	Velocity component in y_b axis              	[m/s]              	 
%   	w   X(3)	Velocity component in z_b axis              	[m/s]
%   	p   X(4)	Body rate component in x_b axis (roll)      	[rad/s]
%   	q   X(5)	Body rate component in y_b axis (pitch)     	[rad/s]
%   	r   X(6)	Body rate component in z_b axis (yaw)       	[rad/s]
%   	q0  X(7)	Quaternion component 1
%   	q1  X(8)	Quaternion component 2
%   	q2  X(9)	Quaternion component 3
%   	q3  X(10)   Quaternion component 4
%   	xe   X(11)  X location in earth ref frame               	[m]
%   	ye   X(12)  Y location in earth ref frame               	[m]
%   	ze   X(13)  Z location in earth ref frame               	[m
%
%   AircraftData: Structure that contains all details of the aircraft.
%
% Outputs:
%   FB_x: x-component of weight force             [N]
%   FB_y: y-component of weight force             [N]
%   FB_z: z-component of weight force             [N]        
%
% Required:
%   Structure file
%   get_rotation_matrix.m
%
% Correct?: YES tested by tom - correct 

function [FB_x,FB_y,FB_z] = Gravity(X, AircraftData)
    
    %Initialise parameters (mass) and (gravity) 
    m = AircraftData.Inertial.m; g = AircraftData.Inertial.g;
    
    %Create 3D vector of Forces 
    F_E = [0;0;m*g];
    
    %Take Quaternion values from X - State Vector and translate it into
    %quaternion vector
    vector(1) = X(7);  vector(2) = X(8); vector(3) = X(9); vector(4) = X(10);
    
    
    %Calculate the rotation matrix/transformation matrix from Body to Earth
    C_BE = get_rotation_matrix(vector);
    
    %Remember that F_B = C_BE*F_E
    %Therefore to get the Body Force, Multiply C_BE by F_E.
    FB = C_BE*F_E;
    
    %Consider components
    FB_x = FB(1); FB_y = FB(2); FB_z = FB(3);
end
