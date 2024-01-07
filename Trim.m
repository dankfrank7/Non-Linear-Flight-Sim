function [X_trim, U_trim] = Trim(X_old, AircraftData)
% AERO3560 Flight Mechanics 1 A3 
% Author: Thomas Ryan
% 
% Info
%   This function uses a Newton-Raphson numerical root finding algorithm to
%   solve for the controls and states of equilibrium steady level flight 
%
% Inputs:
%   X0: Inital State vector [12x1 Column vector] 
%       u      X(1)   Velocity component in x_b axis                [m/s]    
%       v      X(2)   Velocity component in y_b axis                [m/s]                   
%       w      X(3)   Velocity component in z_b axis                [m/s] 
%       p      X(4)   Body rate component in x_b axis (roll)        [rad/s]
%       q      X(5)   Body rate component in y_b axis (pitch)       [rad/s]
%       r      X(6)   Body rate component in z_b axis (yaw)         [rad/s]
%       phi    X(7)   Bank euler angle                              [rad]
%       theta  X(8)   Pitch euler angle                             [rad]
%       psi    X(9)   Yaw euler angle                               [rad]
%       xe     X(10)  X location in earth ref frame                 [m]
%       ye     X(11)  Y location in earth ref frame                 [m]
%       ze     X(12)  Z location in earth ref frame                 [m]
%   U0:
%   AircraftData:
%
% Outputs:
%   X_trim:
%   U_trim:
% 
% Correct? Not tested
    
    % Unpack aircraft data variables
    CLa = AircraftData.Aero.CLa;    % Lift curve slope 
    CLo = AircraftData.Aero.CLo;    % Lift at zero AoA 
    S = AircraftData.Geo.S;         % Area 
    m = AircraftData.Inertial.m;    % mass
    g = AircraftData.Inertial.g;    % gravity
    alt = -X_old(end);                 % Altitude [m]
    W = m*g;                        % Weight Force at wings level
    
    % Convert the X0 state vector from euler to quaternions 
    top = X_old(1:6);
    bot = X_old(10:12);
    quats = e2q(X_old(7:9));
    X0q = [top;quats;bot];
    
    % Calculate the lift coeffficient by assuming steady level flight
    [V, ~, ~] = AeroAngles(X0q);
    [~, Q] = FlowProperties(alt, V);
    CL = W/Q/S;
    
    % Calculate the angle of attack required for this
    alpha_0 = (CL - CLo)/CLa;
    
    % Set up inital geusses for iterations
    % These were defined by the week 9 slides on trim algorithm
    %alpha0  2-3 for high speed, 7-9 for low speed
    delta_t_0 = 0.5;
    delta_r_0 = 0;
    delta_a_0 = 0;
    delta_e_0 = 0;
    U_old = [delta_t_0; delta_e_0; delta_a_0; delta_r_0];
    
    % Set up xbar vector 
    X_bar = [alpha_0; delta_t_0; delta_e_0];
    
    % Set up iteration variables 
    tol = 1e-8;         % error tolerance
    MaxIter = 200;      % max iteration cycles 
    delta = 1e-5;       % perturbation 
    iudot = 1;          % Xdot index for u_dot
    iwdot = 3;          % Xdot inxed for w_dot
    iqdot = 5;          % Xdot index for q_dot
    index = [iudot; iwdot; iqdot];    
    n = length(X_bar);  % Pertubation loop 
    
    % Iteration loop 
    for i = 1:MaxIter
        
        % Step 2 
        % Update Aircraft angle of attack 
        euler = q2e(X_old(7:10));   % Calculate current euler angles
        euler(2) = X_bar(1);    % Update the angle of attack
        X_old(7:10) = e2q(euler);  % Store new euler angles as quats in state
        X_old(7:10) = Normalise(X_old(7:10)); % Normalise 
        
        % Step 3
        % Calculate Xdot 
        X_dot = IterRate(X_old, U_old, AircraftData);
        
        % Extract the perturbation variables from Xdot
        X_dot_bar = X_dot(index);
        
        % Implement pertubation loop 
        for k = 1:n
            
            % State and control pertebation vectors 
            % Reset these for each row 
            X_new = X_old;
            U_new = U_old;
            
            % Firstly perturb alpha, which affects both u and w
            if k == 1
                X_new(1) = cos(X_bar(1)+delta)*V;   % u velocity perturbation
                X_new(3) = sin(X_bar(1)+delta)*V;   % w velocity perturbation
            
            % Secondly perturb delta_t 
            elseif k == 2 
                U_new(1) = X_bar(2)+delta;
            
            % Thirdly perturbn delta_e
            elseif k == 3
                U_new(2) = X_bar(3)+delta;
            end
        
            % Calculate the new state rates from the new state and inputs
            X_dot_new = IterRate(X_new, U_new, AircraftData);
            X_dot_new_bar = X_dot_new(index);
            
            % Assign Jacobian colomn 
            J_mat(:,k) = (X_dot_new_bar - X_dot_bar)./delta;
        end
       
        % Calculate the new X_bar vector
        X_bar_new = X_bar - J_mat\X_dot_bar;
        
        % Calcualte error
        err = abs((X_bar_new-X_bar)./X_bar);
        
        % Check if the vector isconverging 
        if max(err) < tol
            break
           
        % Check if reached max iteration limit 
        elseif i == MaxIter
            warning('Max iteration limit reached - Trim.m')
        end 
        
        % If not converged update the X_bar vector 
        X_bar = X_bar_new;
        
        % Update State vector
        X_old(1) = cos(X_bar_new(1))*V;
        X_old(3) = sin(X_bar_new(1))*V;

        % Update Control Vector 
        U_old(1) = X_bar_new(2);
        U_old(2) = X_bar_new(3);
        
 
    end    
    
    % If converged, then assign the trim deflections 
    X_trim = X_old;
    U_trim = U_old;

   
return
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    