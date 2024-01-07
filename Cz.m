function matrix = Cz(angle)
% This function takes an angle in radians as input and outputs the
% rotation matrix in the z axis
% Authror: Thomas Ryan
matrix = [cos(angle) sin(angle) 0;
          -sin(angle) cos(angle) 0;
          0 0 1];
          
      
return 
