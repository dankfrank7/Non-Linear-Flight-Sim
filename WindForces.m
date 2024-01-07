% AERO3560 Flight Mechanics 1 A3
% Author: Casey Garcia
%
% function [Cfax, Cfaz] = WindForces(X, U, AircraftData, alpha, V, alpha_dot)
%
% Info:
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
%   	ze   X(12)  Z location in earth ref frame               	[m]
%
%
% Outputs:
%
%
% Required .m files:
%
% Correct?: NOT TESTED
function [Cd, CL] = WindForces(X, U, AircraftData, alpha, V, alpha_dot)
    q = X(5); 
    c = AircraftData.Geo.c;
    
    qhat = q*c/(2*V);
    alphahat = alpha_dot*c/(2*V);
    
    Clo = AircraftData.Aero.CLo;
    Clalpha = AircraftData.Aero.CLa;
    Clalphadot = AircraftData.Aero.CLad;
    Clq = AircraftData.Aero.CLq;
    Clde = AircraftData.Aero.CLde;
    
    de = U(2);
    
    CL = Clo + Clalpha*alpha + Clalphadot*alphahat + Clq*qhat + Clde*de;
    Cdo = AircraftData.Aero.Cdo; 
    k = AircraftData.Aero.k; 
    Cd = Cdo + k*CL^2;
end
