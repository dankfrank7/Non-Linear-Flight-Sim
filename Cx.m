function matrix = Cx(angle)
% This function takes an angle in radians as input and outputs the
% rotation matrix in the x axis
% Authror: Thomas Ryan
matrix = [1 0 0;
          0 cos(angle) sin(angle);
          0 -sin(angle) cos(angle)];
      
return 
