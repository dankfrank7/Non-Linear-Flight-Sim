function Thrust = PropForces(X, U, AircraftData, rho)
%this function will convet the propulsicve forces in the body axes
%according  to the simple model 
%inputs are X, U, aircraft data and rho 
%outputs: thrust T 
%get variables from aircraft data - will depend on CG
% Correct
eta = AircraftData.Prop.eta; %propulsive efficiency 
P_maxSL = AircraftData.Prop.P_max; %max power at sea level 
%calculate density ratio (sigma) using SLS rho and rho from input 
rho_sl = 1.22561; %SLS rho
sigma = rho/rho_sl;
%calculate max power at altitude
P_max = P_maxSL*(1.1324*sigma - 0.1324); 
%initialise values for thrust calc 
d_T = U(1); % thrust input control 
V_T = X(1); %Tangential velocity 'u'
[V, alpha, beta] = AeroAngles(X);
%Calcualte thrust
Thrust = eta*P_max*d_T/V;
end 