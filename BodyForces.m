function [Fb_x, Fb_y, Fb_z, Lb, Mb, Nb] = BodyForces(X, U, AircraftData, Cfax, Cfaz, Q, alpha_dot, beta_dot)
% AERO3560 Flight Mechanics 1 A2
% Author: Thomas Ryan, Casey Garcia
%
% Info:
%   This function takes the force coefficients in aerodynamic axis and
%   converts them to the body axis. 
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
%       xe  X(11)   X location in earth ref frame                   [m]
%       ye  X(12)   Y location in earth ref frame                   [m]
%       ze  X(12)   Z location in earth ref frame                   [m]
%
%   U: Control Vector [4x1 Column Vector]
%       delta_T     U(1)    Change in Thrust                        [N]    
%       delta_e     U(2)    Elevator Deflection                     [rad]
%       delta_a     U(3)    Airleron Deflection                     [rad]
%       delta_r     U(4)    Rudder Deflection                       [rad]
%
%   AircraftData: Structure containing aircraft parameters
%   Cfax - CD
%   Cfaz - CL
%   Q
%   alpha_dot
%   beta_dot
%
% Outputs
%   Fax - doesn't include F_t (this is inculded in the StateRates.m func)
%   Fay
%   Faz
%   La
%   Ma
%   Na
%
% Correct?: Not tested

    %% Initialisation
    % Geometric Parameters 
    S = AircraftData.Geo.S;        
    c = AircraftData.Geo.c;       
    b = AircraftData.Geo.b;
    
    % Aircraft State Parameters
    p = X(4);
    q = X(5);
    r = X(6);
    
    % Aircraft Control Parameters 
    delta_e = U(2);
    delta_a = U(3);
    delta_r = U(4);
    
    % Aero angles 
    [V, alpha, beta] = AeroAngles(X);
    
    % Non-Dimensionalised Body Rates
    p_hat = (p*b)/(2*V);
    q_hat = (q*c)/(2*V);
    r_hat = (r*b)/(2*V);
    
    % Non-Dimensionalised Body Rate ROC 
    alpha_dot_hat = (alpha_dot*c)/(2*V);
    beta_dot_hat = (beta_dot*b)/(2*V);
    

    %% Forces
    % Side force Aerodynamic Derivatives 
    Cyb = AircraftData.Aero.Cyb;
    Cybd = AircraftData.Aero.Cybd;
    Cyp = AircraftData.Aero.Cyp;
    Cyr = AircraftData.Aero.Cyr;
    Cyda = AircraftData.Aero.Cyda;
    Cydr = AircraftData.Aero.Cydr;
    
    % Side Force Coefficient 
    Cfay = Cyb*beta+Cybd*beta_dot_hat+Cyp*p_hat+Cyr*r_hat+ Cyda*delta_a + Cydr*delta_r;
    
    % Calculate Forces 
    Fa_x = -Cfax*Q*S;    % x body force (NOT INCLUDING THRUST)
    Fa_y = Cfay*Q*S;    % y force
    Fa_z = -Cfaz*Q*S;    % z force
    
    %% Moments 
    % M Moment Aerodynamic Derivatives
    Cmo = AircraftData.Aero.Cmo;
    Cma = AircraftData.Aero.Cma;
    Cmq = AircraftData.Aero.Cmq;
    Cmad = AircraftData.Aero.Cmad;
    Cmde = AircraftData.Aero.Cmde;
    % N Moment Aerodynamic Derivatives
    Cnb = AircraftData.Aero.Cnb;
    Cnbd = AircraftData.Aero.Cnbd;
    Cnp = AircraftData.Aero.Cnp;
    Cnr = AircraftData.Aero.Cnr;
    Cnda = AircraftData.Aero.Cnda;
    Cndr = AircraftData.Aero.Cndr;
    % L Moment Aerodynamic Derivatives
    Clb = AircraftData.Aero.Clb;
    Clbd = AircraftData.Aero.Clbd;
    Clp = AircraftData.Aero.Clp;
    Clr = AircraftData.Aero.Clr;
    Clda = AircraftData.Aero.Clda;
    Cldr = AircraftData.Aero.Cldr;
    
    % Calculate Moment Coefficients
    Cl = Clb*beta+Clbd*beta_dot_hat+Clp*p_hat+Clr*r_hat+Clda*delta_a+Cldr*delta_r;
    Cm = Cmo+Cma*alpha+Cmad*alpha_dot_hat + Cmq*q_hat + Cmde*delta_e;
    Cn = Cnb*beta+Cnbd*beta_dot_hat+Cnp*p_hat+Cnr*r_hat+ Cnda*delta_a+ Cndr*delta_r;
    
    % Calculate Moments 
    La = Q*Cl*S*b;
    Ma = Q*Cm*S*c;
    Na = Q*Cn*S*b;
    
    %% Convert to body axis 
    % Rotation Matrix 
    Cba = Cy(alpha)*Cz(-beta);
    Forces_body = Cba*[Fa_x;Fa_y;Fa_z];
    Moments_body = Cba*[La; Ma; Na];
    
    % Assign output
    Fb_x = Forces_body(1);
    Fb_y = Forces_body(2);
    Fb_z = Forces_body(3);
    Lb = Moments_body(1);
    Mb = Moments_body(2);
    Nb = Moments_body(3);
    

return
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    