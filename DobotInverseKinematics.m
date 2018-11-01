function [viable_angles] = DobotInverseKinematics(p0T, previous_angles)
% finds viable joint angles (in range -pi to pi) given p0T
% INPUTS:
%   p0T - xyz desired coordinates, CALIBRATED
%   previous_angles - to find single viable angle, otherwise will return all
% OUTPUTS:
%   viable_angles - all angles that can lead to position p0T

global p01 p12 p23 p3T

addpath('general-robotics-toolbox-master');
assert(length(p0T) == 3, 'Error: You need to give desired xyz coordinates')

ex = [1;0;0]; ey = [0;1;0]; ez = [0;0;1];

viable_angles = [];

% SUBPROBLEM4
%   theta = subproblem4(p, q, k, d)
%   solve for theta when static displacement from rotation axis from
%   d=p'*rot(k,theta)*q
p = ex;
q = p0T-p01;
k = ez;
d = ex'*(p12 + p23 + p3T);
q1vec = mod(-subproblem4(p,q,k,d)+pi,2*pi)-pi; % fit to range (-pi,pi)

for q1 = q1vec'
    % SUBPROBLEM3
    %   theta = subproblem3(p, q, k, d)
    %   
    %   solve for theta in an elbow joint according to
    %   || q + rot(k, theta)*p || = d
    p = -p3T;
    q = p23;
    k = ex;
    d = norm(rot(ez,-q1)*(p0T-p01)-p12);
    q2minus3vec = mod(subproblem3(p,q,k,d)+pi,2*pi)-pi;
    
    for q2minus3 = q2minus3vec'
        % SUBPROBLEM1
        %   theta=subproblem1(p, q, k)
        %
        % solve for theta according to
        %   q = rot(k, theta)*p
        p = p23+rot(ex,q2minus3)*p3T;
        q = rot(ez,-q1)*(p0T-p01)-p12;
        k = ex;
        q2 = mod(-subproblem1(p,q,k)+pi,2*pi)-pi;
        
        q3 = mod(-q2minus3+q2+pi,2*pi)-pi;
        
        viable_angles = [viable_angles [q1;q2;q3]];
    end
end

if ~isempty(previous_angles)
    min_norm = Inf;
    min_norm_angles = Inf;
    for i = 2:length(viable_angles)
        if norm(viable_angles(:,i)-previous_angles) < min_norm
            min_norm = norm(viable_angles(:,i)-previous_angles);
            min_norm_angles = viable_angles(:,i);
        end
    end
    viable_angles = min_norm_angles;
end

end