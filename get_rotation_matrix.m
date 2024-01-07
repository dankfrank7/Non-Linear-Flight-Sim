% AERO3560 Flight Mechanics 1 A3
% Author: Jasraj Chouhan 
%
% function C_BE = get_rotation_matrix(vector)
%
% Info:
%   This function returns the rotation matrix for the Earth to Body frame
%   Note: q is vector of quaternion values
%   
%   Note: C_BE is a 3x3 matrix going from E to B
%
% Inputs:
%      vector = [q0 q1 q2 q3]            
%
% Outputs:
%   C_BE = 3x3 Rotation Matrix from Body frame to Earth frame.         
%
% Required:
%   Input Quaternion Value
% Correct?: NOT TESTED
function C_BE = get_rotation_matrix(vector)
%determine all values in this transformation matrix
l_1 = vector(1)^2 + vector(2)^2 - vector(3)^2 - vector(4)^2;
l_2 = 2*(vector(2)*vector(3) + vector(1)*vector(4));
l_3 = 2*(vector(2)*vector(4) - vector(1)*vector(3));
m_1 = 2*(vector(2)*vector(3) - vector(1)*vector(4));
m_2 = vector(1)^2 - vector(2)^2 + vector(3)^2 - vector(4)^2;
m_3 = 2*(vector(3)*vector(4) + vector(1)*vector(2));
n_1 = 2*(vector(1)*vector(3) + vector(2)*vector(4));
n_2 = 2*(vector(3)*vector(4) + vector(1)*vector(2));
n_3 = vector(1)^2 - vector(2)^2 - vector(3)^2 + vector(4)^2;
C_BE = [l_1 l_2 l_3;m_1 m_2 m_3;n_1 n_2 n_3];
end
