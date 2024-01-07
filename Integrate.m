function X_t = Integrate(X, U, AircraftData, dt) 
%description of function: Use a 4th order Runge-Kutta integration routine
%to compute the next state for a chosen timestep size. The magnitude of the
%gradient of X will be calculated at 4 different points over the timestep
%then averaged to provided greater numerical accuracy than the Euler method
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
%   dt: timestep 
%
%   AircraftData: Structure containing aircraft parameters
%
% Outputs:
%       X_t: State vector after Runge -Kutta integration over 1 time step[13x1 Column vector] 
%       same order and units as X
%CORRECT? - not tested, might need to normalise the quarternions as well!!!

%normalise quarternions of the input vector 
X(7:10) = X(7:10)/norm(X(7:10));

% Step 1 Evaluate xdot1  at start of timestep (tk) and evaluate increment
X_dot1 = IterRate(X, U, AircraftData);
A_n = X_dot1*dt;

% Step 2 - Evaluate xdot2 at tk +dt/2
X_2 = X + A_n/2; %state vector at middle of timestep  
X_2(7:10) = X_2(7:10)/norm(X_2(7:10)); %normalise quarternions of the vector 
X_dot2 = IterRate(X_2, U, AircraftData); %rate vector found at mid point 
B_n = X_dot2*dt; %improved increment 

%Step 3 - Evaluate xdot3 at tk +dt/2 using previous increment for better
%accuracy 
X_3 = X + B_n/2; %state vector at this point 
X_3(7:10) = X_3(7:10)/norm(X_3(7:10)); %normalise quarternions of the vector 
X_dot3 = IterRate(X_3, U, AircraftData); %state rate vector at this point
C_n = X_dot3*dt; %improved increment 

%Step 4 - evaluate xdot4 at end of timestep 
X_4 = X + C_n; %state vec 
X_4(7:10) = X_4(7:10)/norm(X_4(7:10)); %normalise quarternions of the vector 
X_dot4 = IterRate(X_4, U, AircraftData); %state rate vector 
D_n = X_dot4*dt; %last increment 

%step 5 - predict total increment as a weighted average and evaluate X at the
%next timestep
X_t = X + 1/6 *(A_n + 2*B_n + 2*C_n + D_n);
X_t(7:10) = X_t(7:10)/norm(X_t(7:10)); %normalise quarternions of the new vector 
end 
