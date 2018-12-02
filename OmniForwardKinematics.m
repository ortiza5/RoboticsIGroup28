function [p]=OmniForwardKinematics(q)
% finds p0T given joint angles q
% INPUTS:
%   q - 3x1 vector of joint angles IN DEGREES
%   p4T - position from O4 to pen tip, needs calibration
% OUTPUTS:
%   p0T - [x;y;z] of resulting coordinates
%x=[1;0;0];y=[0;1;0];z=[0;0;1];

assert(length(q) == 3, 'Error: You need to give 3 angles')
ex=[1;0;0]; ey=[0;1;0]; ez=[0;0;1]; zv=[0;0;0];

q = deg2rad(q);
q1 = q(1); q2 = q(2); q3 = q(3);
l1 = 0.135; l2 = 0.135; l3 = 0.025; l4 = 0.17;

x=-sin(q1)*(l1*cos(q2)+l2*sin(q3));
z=l3-l2*cos(q3)+l1*sin(q2);
y=l4+cos(q1)*(l1*cos(q2)+l2*sin(q3));

p=[x;y;z];
end