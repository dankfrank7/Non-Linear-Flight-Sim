function X_dot = StateRates( X, U, AircraftData, alpha_dot, beta_dot)
% AERO3560 Flight Mechanics 1 A3
% Author: Thomas Ryan
% 
%  function X_dot = StateRates( X, U, AircraftData, alpha_dot, beta_dot)
% 
% Info: 
%   This function calculates the time derivatives of of each state variable
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
%   alpha_dot: rate of change of angle of attack                    [rad/s]
%   beta_dot:  rate of change of sideslip angle                     [rad/s]
%
% Output:
%   X_dot: State vector rate of change [13x1 Column vector] 
%       u_dot   X(1)    RoC Velocity component in x_b axis          [m/s^2]    
%       v_dot   X(2)    RoC Velocity component in y_b axis          [m/s^2] 
%       w_dot   X(3)    RoC Velocity component in z_b axis          [m/s^2] 
%       p_dot   X(4)    RoC Body rate component in x_b axis (roll)[rad/s^2]
%       q_dot   X(5)    RoC Body rate component in y_b axis (pitch)[rad/s^2]
%       r_dot   X(6)    RoC Body rate component in z_b axis (yaw) [rad/s^2]
%       q0_dot  X(7)    RoC Quaternion component 1                   [/s]
%       q1_dot  X(8)    RoC Quaternion component 2                   [/s]
%       q2_dot  X(9)    RoC Quaternion component 3                   [/s]
%       q3_dot  X(10)   RoC Quaternion component 4                   [/s]
%       xe_dot  X(11)  RoC X location in earth ref frame             [m/s]
%       ye_dot  X(12)  RoC Y location in earth ref frame             [m/s]
%       ze_dot  X(12)  RoC Z location in earth ref frame             [m/s]
%
% Required .m files: BodyForces.m, Gravity.m, PropForces.m, WindForces.m,
%                    DCM_quat.m
%
% Correct?: VERY CLOSE TO PERFECT

    %% Extract state variables
    u = X(1);
    v = X(2);
    w = X(3);
    p = X(4);
    q = X(5);
    r = X(6);
    q0 = X(7);
    q1 = X(8);
    q2 = X(9);
    q3 = X(10);

    % Extract flight data
    Ixx = AircraftData.Inertial.Ixx;         
    Iyy = AircraftData.Inertial.Iyy;      
    Izz = AircraftData.Inertial.Izz;      
    Ixz = AircraftData.Inertial.Ixz;
    m   = AircraftData.Inertial.m;   

    % Calculate angular rate coefficients (Lec 2A Slide 34)
    C0 = Ixx*Izz - Ixz.^2;
    C1 = Izz/C0;
    C2 = Ixz/C0;
    C3 = C2*(Ixx-Iyy+Izz);
    C4 = C1*(Iyy-Izz)-C2*Ixz;
    C5 = 1/Iyy;
    C6 = C5*Ixz;
    C7 = C5*(Izz-Ixx);
    C8 = Ixx/C0;
    C9 = C8*(Ixx-Iyy)+C2*Ixz;
    
    % Calculate aero values 
    [V, alpha, beta] = AeroAngles(X);

    %% Calculate Body, Wind, Prop and Gravity forces from other functions
    % CHECK THESE FUNCTIONS
    % Flow properties
    [rho,Q] = FlowProperties(-X(13),V); %Correct (slight difference doesnt produce measurable error)
    % Wind forces
    [Cfax, Cfaz] = WindForces(X, U, AircraftData, alpha, V, alpha_dot); %Correct
    % Body Forces
    [Fbx, Fby, Fbz, La, Ma, Na] = BodyForces(X, U, AircraftData, Cfax, Cfaz, Q, alpha_dot, beta_dot); % Correct 
    % Gravity forces
    [Fgx, Fgy, Fgz] = Gravity(X, AircraftData); %Correct
    % Prop forces
    Thrust = PropForces(X, U, AircraftData, rho); % Correct 
    
    % Convert Thrust from aero axis to body axis
    Cz_ba = Cz(-beta);
    Cy_ba = Cy(alpha);
    Cba = Cy_ba*Cz_ba;
    Thrust_body = Cba*[Thrust;0;0];

    %% Calculate the flight body vector rates of change (Lec 2A Sldie 34)
    u_dot = r.*v - q.*w + Fgx/m + (Fbx + Thrust_body(1))./m;
    v_dot = p.*w-r.*u+Fgy/m+Fby/m;
    w_dot = q.*u-p.*v+Fgz/m + (Fbz-Thrust_body(3))/m;

    % Calculate the body rate rates of change (Lec 2A Slide 34)
    p_dot = C3.*p.*q+C4.*q.*r+C1.*La+C2.*Na;
    q_dot = C7.*p.*r-C6.*(p.^2-r.^2)+C5.*Ma;
    r_dot = C9.*p.*q-C3.*q.*r+C2.*La+C8.*Na;

    % Calculate Quaternion Rates
    q0_dot = (q1.*p+q2.*q+q3.*r)/(-2);
    q1_dot = (q0.*p-q3.*q+q2.*r)/2;
    q2_dot = (q3.*p+q0.*q-q1.*r)/2;
    q3_dot = (q2.*p-q1.*q-q0.*r)/(-2);

    % Calculate transformation matrix from body to earth coordinates
    Cbe = DCM_Quat(q0,q1,q2,q3);    % This is correct 
    earth_pos = Cbe\[u;v;w];    % Invert the Cbe matrix to get Ceb

    % Define X_dot output
    X_dot = [u_dot;
             v_dot;
             w_dot;
             p_dot;
             q_dot;
             r_dot;
             q0_dot;
             q1_dot;
             q2_dot;
             q3_dot;
             earth_pos(1);
             earth_pos(2);
             earth_pos(3)];
return










