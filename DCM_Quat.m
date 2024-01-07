function Cbe = DCM_Quat(q0,q1,q2,q3)
% AERO3560 Flight Mechanics 1 A3
% Author: Thomas Ryan
% 
% function Cbe = DCM_Quat(q1,q2,q3,q4)
% 
% Info: 
%   Transforms a four element unit quat vector into a 3x3 direction
%   cosine matrix. See
%   https://au.mathworks.com/help/aeroblks/quaternionstodirectioncosinematrix.html
%   for more details.
%
% Inputs:
%   q0:     Quaternion component 1
%   q1:     Quaternion component 2
%   q2:     Quaternion component 3
%   q3:     Quaternion component 4
%
% Outputs: 
%   Cbe: 3x3 Direct Cosine Matrix from earth to body
%
% Other .m required: None
%
% Correct?: NOT TESTED

    % Components of the matrix
    c1 = q0^2+q1^2-q2^2-q3^2;
    c2 = 2*(q1*q2+q0*q3);
    c3 = 2*(q1*q3-q0*q2);
    c4 = 2*(q1*q2-q0*q3);
    c5 = q0^2-q1^2+q2^2-q3^2;
    c6 = 2*(q2*q3+q0*q1);
    c7 = 2*(q0*q2+q1*q3);
    c8 = 2*(q2*q3-q0*q1);
    c9 = q0^2-q1^2-q2^2+q3^2;
    
    % Create the matrix 
    Cbe = [c1, c2, c3;
           c4, c5, c6;
           c7, c8, c9];
return
       