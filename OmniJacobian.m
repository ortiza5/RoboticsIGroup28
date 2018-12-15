function [J]=OmniJacobian(q)
% finds jacobian of Phantom Omni given joint angles q
% Based of: http://www.academia.edu/22070087/PHANToM_OMNI_Haptic_Device_Kinematic_and_Manipulability
% INPUTS:
%   q - 3x1 vector of joint angles IN DEGREES
% OUTPUTS:
%   J - 3x3 jacobian matrix

% Setting up parameters
assert(length(q) == 3, 'Error: You need to give 3 angles')
q = deg2rad(q);

l1 = 0.135; l2 = 0.135; l3 = 0.025; l4 = 0.17;

c1 = cos(q(1)); c2 = cos(q(2)); c3 = cos(q(3));
s1 = sin(q(1)); s2 = sin(q(2)); s3 = sin(q(3));

% Computing cells of jacobian matrix
J11 = -(l1*c1*c2 + l2*s3*c1);
J12 = l1*s1*s2;
J13 = -l2*c3*s1;

J21 = 0;
J22 = l1*c2;
J23 = l2*s3;

J31 = -l1*c2*s1;
J32 = -l1*s2*c1;
J33 = l2*c3*c1;

% Output
J = [J11 J12 J13;J31 J32 J33;J21 J22 J23];
end