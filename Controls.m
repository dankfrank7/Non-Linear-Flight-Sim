
% 
% AERO3560 - Flight Mechanics - 1 - A3
% Author: Jasraj Chouhan 
% 
% Function: Controls(U, current_timestep, U_filter, T_filter);
% 
% Info:
%   This function generates the control inputs for the current timestep
%   
% Inputs:
%    U: Vector that contains all aircraft control settings
%                   - de_t = U(1)           Throttle 
%                   - de_e = U(2)    (rad)  Elevator
%                   - de_a = U(3)    (rad)  Aileron
%                   - de_r = U(4)    (rad)  Rudder
% 
%   Time: Time vector used for the entire simulation
% 
%  U_filter: From Control GUI for Red Bull
% 
%  T_filter: From Control GUI for Red Bull
% 
% Outputs:
%   U: Vector that contains new aircraft control settings. Same order as U
%   above.
% Required:
%   Input: Control vector, Timestep, U_filter, T_filter from Control GUI.
% 
% Correct?: NOT TESTED
function U_new = Controls(U_trim, current_timestep, U_filter, T_filter)
    
    %Access the first 4 rows of the GUI output(note the deflections are added on) 
    U_filter = U_filter(1:4,:) + U_trim;
    
    %Initialise the trim vector for control settings
    U_new = U_trim;
    
    %loop through the t of inputs
    if current_timestep <= T_filter(end)
        
        % Set up while loop counter 
        i = 1;
        
        % Determine the instantaneous position of the aircraft. 
        while i <= length(T_filter)
            
            % Find the index i from the filter vector that matches the time
            % in the integration time step to within 5% 
            if current_timestep > 0.95*T_filter(i) && current_timestep < 1.05*T_filter(i)
                break
            end
            
            % Update while loop
            i = i + 1;
        end
        
        % Replace only the non-zero elements of the control vector into the
        % trim vector
        U_new = U_filter(:,i);
        
        % And exit from the function 
        return
    end
end
