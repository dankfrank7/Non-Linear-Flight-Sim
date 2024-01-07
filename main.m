% AERO3560 Flight Mechanics 1 A3
% Author: Thomas Ryan

% Start with a clear workspace
clear; clc; close all

%% Initialisation 
% Assign U_filter and T_filter error flags
U_in = -1;
T_in = -1;

% Select flight condition and CG location 
[U_filter,T_filter, AircraftData, X_init, U_init, flag] = Initialisation(U_in ,T_in);

if flag 
    run Control_GUI
    input('Enter deflections in RADIANS [Press ENTER when control vector loaded]')
end

% Do you want plots generated?
toPlot = input('Would you like to Plot data? (1 = yes, 0 = no): ');

% Calculat initial trim setting 
[X_trim, U_trim] = Trim(X_init, AircraftData);



%% Integration Loop
% Set up temporal vector
t_end  = input('Simulation time (s): ');
dt = 0.01;
t_vec = 0:dt:t_end;
n = length(t_vec);

% Preallocate Memory 
X = zeros(13, n);
U = zeros(4, n);

% Set initial condition 
X(:,1) = X_trim;
U(:,1) = U_trim;

% Loop through time 
for i = 2:n
    i
    % The aircraft maintains trimed controls for 1 second 
    if t_vec(i) <= 1
        
        % Control vector stays constant 
        U(:,i) = U_trim;
        
        % Calculate the state at current time from previous time step and
        % store it the result vector
        X(:,i) = Integrate(X(:,i-1), U(:,i), AircraftData, dt);
        
    else
        
        % New control vector 
        %U(:,i) = Controls(U_trim, t_vec(i), U_filter, T_filter);
        U(:,i) = Controls(U_trim, t_vec(i), U_filter, T_filter);
        
        % Calculate the state at current time from previous time step and
        % new control vector and store it in result vector
        X(:,i) = Integrate(X(:,i-1), U(:,i), AircraftData, dt); 
    end
end
clc
disp(strcat("Integration complete after ",string(i)," iterations."))
if toPlot 
    PlotData(t_vec,X,U) 
end