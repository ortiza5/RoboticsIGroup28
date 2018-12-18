function p0T = DobotForwardKinematics(q)
% finds p0T given joint angles q
% INPUTS:
%   q - 3x1 vector of joint angles IN DEGREES
%   p4T - position from O4 to pen tip, needs calibration
% OUTPUTS:
%   p0T - [x;y;z] of resulting coordinates

assert(length(q) == 3, 'Error: You need to give 3 angles')
ex=[1;0;0]; ey=[0;1;0]; ez=[0;0;1]; %zv=[0;0;0];

q = deg2rad(q);
q1 = q(1); q2 = q(2); q3 = q(3);
l1 = 103; l2 = 135; l3 = 160; l4 = 50; l5 = 75;

% p0T = l1*ez + rot(ez,q1)*rot(-ex,q2)*l2*ez + rot(ez,q1)*rot(-ex,q3)*l3*ey + rot(ez,q1)*[l4;0;-l5];
p0T = l1*ez + rot(ez,q1)*rot(ey,q2)*l2*ez + rot(ez,q1)*rot(ey,q3)*l3*ex + rot(ez,q1)*[l4;0;-l5];
end

% function p0T = DobotForwardKinematics(q)
% % finds p0T given joint angles q
% % INPUTS:
% %   q - 3 joint angles
% % OUTPUTS:
% %   p0T - xyz desired coordinates, NOT CALIBRATED
% 
% global p01 p12 p23 p3T
% 
% addpath('general-robotics-toolbox-master');
% 
% ex=[1;0;0]; ey=[0;1;0]; ez=[0;0;1]; zv=[0;0;0];
% h1 = ez; h2 = -ex; h3 = -ex;
% 
% p0T = p01 + rot(h1,q(1))*(p12 + rot(h2,q(2))*p23 + rot(h3,q(3))*p3T);
% end

% copied from team 14
% function [ p, R ] = fwdkindobot( q1, q2, q3 , d)
% %fwdkindobot Outputs rotation and distance from origin
% %   from Dobot's joint angles
% ex = [1; 0; 0];
% ey = [0; 1; 0];
% ez = [0; 0; 1];
% 
% %all lengths are in mm
% l1 = 103;
% l2 = 135;
% l3 = 160;
% Lg = 56;  %length from point T to q4 rotation
% %d = 115;  %pen length
% 
% R = rot(ez, q1)*rot(ex, q2)*rot(ex, q3);
% p = l1*ez + rot(ez,q1)*rot(ey,q2)*l2*ez + rot(ez,q1)*rot(ey,q2)*rot(ey,q3)*l3*ex+[Lg*cos(q1);Lg*sin(q1);-d];
% end