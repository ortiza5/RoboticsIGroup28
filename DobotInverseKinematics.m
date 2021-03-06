function [viable_angles] = DobotInverseKinematics(p0T, previous_angles)
% finds viable joint angles (in range -pi to pi) given p0T
% INPUTS:
%   p0T - [x;y;z] desired coordinates
%   previous_angles - to find single viable angle, otherwise will return all
% OUTPUTS:
%   viable_angles - all angles that can lead to position p0T (only one if
%   previous_angles was given), IN DEGREES

assert(length(p0T) == 3, 'Error: You need to give desired xyz coordinates')

ex = [1;0;0]; ey = [0;1;0]; ez = [0;0;1];
l1 = 103; l2 = 135; l3 = 160; l4 = 50; l5 = 75;

viable_angles = [];

% SUBPROBLEM4
%   theta = subproblem4(p, q, k, d)
%   solve for theta when static displacement from rotation axis from
%   d=p'*rot(k,theta)*q
%   ey'*rot(ez,q1)'*(p0T - l1*ez) = ey'*l2*ez + ey'*l3*ey + ey'*[l4;0;-l5]
d = ey'*l2*ez + ey'*l3*ex + ey'*[l4;0;-l5];
p = ey;
k = ez;
q = p0T - l1*ez;

q1vec = -subproblem4(p,q,k,d);
for q1 = q1vec'
    % SUBPROBLEM3
    %   theta = subproblem3(p, q, k, d)
    %   
    %   solve for theta in an elbow joint according to
    %   || q - rot(k, theta)*p || = d
    %   || l2*ez + rot(ey,q3-q2)*l3*ex || = || rot(ez,q1)'*(p0T - l1*ez) - [l4;0;-l5] ||
    q = l2*ez;
    k = ey;
    p = -l3*ex;
    d = norm(rot(ez,q1)'*(p0T - l1*ez) - [l4;0;-l5]);
    q3minus2vec = mod(subproblem3(p,q,k,d)+pi,2*pi)-pi; % fit to range [-pi,pi)
   
    for q3minus2 = q3minus2vec'
        % SUBPROBLEM1
        %   theta=subproblem1(p, q, k)
        %
        % solve for theta according to
        %   q = rot(k, theta)*p
        %   rot(ez,q1)'*(p0T - l1*ez) - [l4;0;-l5] = rot(ey,q2)*( l2*ez + rot(ey,q3-q2)*l3*ex )
        p = l2*ez + rot(ey,q3minus2)*l3*ex;
        q = rot(ez,q1)'*(p0T - l1*ez) - [l4;0;-l5];
        k = ey;
        q2 = subproblem1(p,q,k);

        q3 = q3minus2+q2;
        
        viable_angles = [viable_angles [q1;q2;q3]];
    end
end

viable_angles = rad2deg(viable_angles);
viable_angles = mod(viable_angles+180,360)-180; % fit to range [180,180)

% only keep values in allowed range
viable_angles_temp = [];
for i = 1:size(viable_angles,2)
    q1 = viable_angles(1,i);
    q2 = viable_angles(2,i);
    q3 = viable_angles(3,i);
    
    if q1 >= -135 && q1 <= 135
        if q2 >= -5 && q2 <= 85
            if q3 >= -10 && q3 <= 90
                viable_angles_temp = [viable_angles_temp [q1;q2;q3]];
            end
        end
    end
end

viable_angles = viable_angles_temp;

if nargin == 2
    min_norm = Inf;
    min_norm_angles = Inf;
    for i = 1:size(viable_angles,2)
        if norm(viable_angles(:,i)-previous_angles) < min_norm
            min_norm = norm(viable_angles(:,i)-previous_angles);
            min_norm_angles = viable_angles(:,i);
        end
    end
    viable_angles = min_norm_angles;
    
    if(min_norm_angles == Inf)
        viable_angles = previous_angles;
        disp('outside bounds')
    end
end

end

% function [viable_angles] = DobotInverseKinematics(p0T, previous_angles)
% % finds viable joint angles (in range -pi to pi) given p0T
% % INPUTS:
% %   p0T - xyz desired coordinates, CALIBRATED
% %   previous_angles - to find single viable angle, otherwise will return all
% % OUTPUTS:
% %   viable_angles - all angles that can lead to position p0T
% 
% global p01 p12 p23 p3T
% 
% addpath('general-robotics-toolbox-master');
% assert(length(p0T) == 3, 'Error: You need to give desired xyz coordinates')
% 
% -ey = [1;0;0]; ex = [0;1;0]; ez = [0;0;1];
% 
% viable_angles = [];
% 
% % SUBPROBLEM4
% %   theta = subproblem4(p, q, k, d)
% %   solve for theta when static displacement from rotation axis from
% %   d=p'*rot(k,theta)*q
% p = -ey;
% q = p0T-p01;
% k = ez;
% d = -ey'*(p12 + p23 + p3T);
% q1vec = mod(-subproblem4(p,q,k,d)+pi,2*pi)-pi; % fit to range (-pi,pi)
% 
% for q1 = q1vec'
%     % SUBPROBLEM3
%     %   theta = subproblem3(p, q, k, d)
%     %   
%     %   solve for theta in an elbow joint according to
%     %   || q + rot(k, theta)*p || = d
%     p = -p3T;
%     q = p23;
%     k = -ey;
%     d = norm(rot(ez,-q1)*(p0T-p01)-p12);
%     q2minus3vec = mod(subproblem3(p,q,k,d)+pi,2*pi)-pi;
%     
%     for q2minus3 = q2minus3vec'
%         % SUBPROBLEM1
%         %   theta=subproblem1(p, q, k)
%         %
%         % solve for theta according to
%         %   q = rot(k, theta)*p
%         p = p23+rot(-ey,q2minus3)*p3T;
%         q = rot(ez,-q1)*(p0T-p01)-p12;
%         k = -ey;
%         q2 = mod(-subproblem1(p,q,k)+pi,2*pi)-pi;
%         
%         q3 = mod(-q2minus3+q2+pi,2*pi)-pi;
%         
%         viable_angles = [viable_angles [q1;q2;q3]];
%     end
% end
% 
% if ~isempty(previous_angles)
%     min_norm = Inf;
%     min_norm_angles = Inf;
%     for i = 2:length(viable_angles)
%         if norm(viable_angles(:,i)-previous_angles) < min_norm
%             min_norm = norm(viable_angles(:,i)-previous_angles);
%             min_norm_angles = viable_angles(:,i);
%         end
%     end
%     viable_angles = min_norm_angles;
% end
% 
% end