function angles = q2e(quat)
% This function takes a quaternian and converts it to euler angels (IN
% RADIANS)
% 
% INPUTS 
% quat          - 4xn  vector containing quaternion [q0; q1; q2; q3]
% 
% OUPUTS 
% phi           - roll angle (earth to body) [rad]
% theta         - pitch angle (earth to body) [rad]
% psi           - yaw angle (earth to body) [rad]
% 
% Author: Thomas Ryan

    % Check to see if the input is in the right format 
    if size(quat,1) ~= 4 
        disp(['Input quat dimension is: ',string(size(quat))])
        error('TOM WROTE THIS Angle vector is not in the correct format')
    end 
    
    % Loop through every quaternion 
    for i = 1:size(quat,2)
        % Index the input vector to assign variables for each quaternion 
        q0 = quat(1,i);
        q1 = quat(2,i);
        q2 = quat(3,i);
        q3 = quat(4,i);

        % Calculate euler angle values 
        phi = atan2((q2*q3+q0*q1), (q0^2+q3^2-0.5));
        theta = atan2((q0*q2-q1*q3), ((q0^2+q1^2-0.5)^2+(q1*q2+q0*q3)^2)^0.5);
        psi = atan2((q1*q2+q0*q3), (q0^2+q1^2-0.5));
        angles(:,i) = [phi; theta; psi];
    end
end
