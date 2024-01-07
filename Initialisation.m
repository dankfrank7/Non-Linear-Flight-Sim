
%AERO3560 Assignment 3 
% info: Initialise all aircraft coefficients, mass properties, reference dimensions, and flight conditions
%function 
%inputs: file doesnt take any inputs 
%outputs: flight data at each cg location 
function [U_filter,T_filter, AircraftData, X_init, U_init, flag] = Initialisation(U_in ,T_in)
%load parameters at nominal 
flag = 0;
U_filter = U_in;
T_filter = T_in;
start = 28;
CG1_data = aero3560_LoadFlightDataPC9_nominalCG1();
%load paramaters at second location
CG2_data = aero3560_LoadFlightDataPC9_CG2();
cg = input('Enter a CG location (1 == NominalCG, 2 = CG2): ');
vel = input('Enter a flight velocity (1 == 100 Kn, 2 = 180 Kn): ');
mode = strcat(string(cg), string(vel));
switch mode
    case "11"
        load ICs_PC9_nominalCG1_100Kn_1000ft
        AircraftData = CG1_data;
        
    case "12"
        load ICs_PC9_nominalCG1_180Kn_1000ft
        AircraftData = CG1_data;
        
    case "21" 
        load ICs_PC9_CG2_100Kn_1000ft
        AircraftData = CG2_data;
        
    case "22"
        load ICs_PC9_CG2_180Kn_1000ft
        AircraftData = CG2_data;
        
    otherwise 
        error('Incorrect CG or Vel input')
end

%Determine which simulation we want to represent
sim = input('Enter a Simulation Condition (1 == Elevator, 2 = Aileron, 3 = Rudder, 4 = Red Bull Race, 5 = Maneuvre Testing): ');
mode = string(sim);
%impulse lasts for 0.5 seconds
dt = 0.01;

switch mode
    case "1"
        %create elevator deflection
        T_filter = (1+dt):dt:1.50;
        U_filter = zeros(5,length(T_filter));
        deflection = deg2rad(input('Enter Elevator Deflection (deg): '));
        U_filter(2,2:end-1) = ones(1,length(T_filter)-2)*deflection;
        
    case "2"
        %create aileron deflection 
        T_filter = (1+dt):dt:1.50;
        U_filter = zeros(5,length(T_filter));
        deflection = deg2rad(input('Enter Airleron Deflection (deg): '));
        U_filter(3,2:end-1) = ones(1,length(T_filter)-2)*deflection;
        
    case "3" 
        %create rudder deflection
        T_filter = (1+dt):dt:1.50;
        U_filter = zeros(5,length(T_filter));
        deflection = deg2rad(input('Enter Rudder Deflection (deg): '));
        U_filter(4,2:end-1) = ones(1,length(T_filter)-2)*deflection;
        
    case "4"
        %need to add control GUI
        % RUN control GUI here? Or maybe we just store the vectors in
        % workspace and then load them here
        X0(12) = -20;   % Set the initial condition to the right altitude
        X0(10) = start; % For testing startint at position 1
        U_temp = load('U_RedBull.mat');
        U_filter = U_temp.U_filter;
        T_temp = load('T_RedBull.mat');
        T_filter = T_temp.T_filter;
        disp("To Complete Race course:  CG1, 100 KEAS, Tmax = 50")
    case "5"
        % Maneuvre Trial
        % Run the control GUI
        flag = 1;
    otherwise 
        error('Incorrect program input.')
end
X_init = Convert2Quat(X0);
U_init = U0;
end 
