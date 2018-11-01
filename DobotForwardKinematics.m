function p0T = DobotForwardKinematics(q)
% finds p0T given joint angles q
% INPUTS:
%   q - 3 joint angles
% OUTPUTS:
%   p0T - xyz desired coordinates, NOT CALIBRATED

global p01 p12 p23 p3T

addpath('general-robotics-toolbox-master');

ex=[1;0;0]; ey=[0;1;0]; ez=[0;0;1]; zv=[0;0;0];
h1 = ez; h2 = -ex; h3 = -ex;

p0T = p01 + rot(h1,q(1))*(p12 + rot(h2,q(2))*p23 + rot(h3,q(3))*p3T);
end