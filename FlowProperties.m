% alt must be in meters
% V must be in m/s
function [rho,Q] = FlowProperties(alt,V)
    if alt >= 25000
        T = -131.21 + 0.00299*alt;
        P = 2.488*((T+273.1)/216.6)^-11.388;
    elseif  alt >= 11000 && alt < 25000
        T = -56.46;
        P = 22.65*exp(1.73-0.000157*alt);
    else
        T = 15.04 - 0.00649*alt;
        P = 101.29*((T+273.1)/288.08)^5.256;
    end
    
    rho = P/(0.2869*(T+273.1));
    Q = 0.5*rho*V^2;
    
end
