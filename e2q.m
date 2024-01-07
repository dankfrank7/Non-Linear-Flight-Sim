function quat = e2q(angles)
% This function takes three euler angles and converts them to quarternions
%
% INPUTS 
% angles        - 3xn  vector ortientation angles [phi; theta; psi]
% phi           - roll angle (earth to body) [rad]
% theta         - pitch angle (earth to body) [rad]
% psi           - yaw angle (earth to body) [rad]
% OUTPUTS
% q0            - quarternion 1
% q1            - quarternion 2
% q2            - quarternion 3
% q3            - quarternion 4
% 
% Author: Thomas Ryan

     % Check to see if the input is in the right format 
     if size(angles,1) ~= 3 
         disp(['Input angle dimension is: ',string(size(angles))])
         error('TOM WROTE THIS Angle vector is not in the correct format')
     end 
    
    % Loop throughe every set of euler angles
    for i = 1:size(angles,2)
        % Divide all angles by two 
        phi2 = angles(1,i)/2;
        theta2 = angles(2,i)/2;
        psi2 = angles(3,i)/2;

        % Calculate quarternions as per lec 2A 
        q0 = cos(psi2)*cos(theta2)*cos(phi2) + sin(psi2)*sin(theta2)*sin(phi2);
        q1 = cos(psi2)*cos(theta2)*sin(phi2) - sin(psi2)*sin(theta2)*cos(phi2);
        q2 = cos(psi2)*sin(theta2)*cos(phi2) + sin(psi2)*cos(theta2)*sin(phi2);
        q3 = -cos(psi2)*sin(theta2)*sin(phi2) + sin(psi2)*cos(theta2)*cos(phi2);
        quat(:,i) = [q0; q1; q2; q3];
    end
end 