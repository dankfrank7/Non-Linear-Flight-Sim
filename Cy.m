function matrix = Cy(angle)
% This function takes an angle in radians as input and outputs the
% rotation matrix in the y axis
% Authror: Thomas Ryan
matrix = [cos(angle) 0 -sin(angle);
          0 1 0;
          sin(angle) 0 cos(angle)];
          
      
return 
