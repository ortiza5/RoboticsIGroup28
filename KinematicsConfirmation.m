clc
close all

global p01 p12 p23 p3T

p01 = [0;0;0];
p12 = [0;0;0];
p23 = [0;0;13.5];
p3T = [0;16;0];

% % visualize the work space
% size = 10000;
% q1max = 135; q1min = -135;
% q2max = 85; q2min = -5;
% q3max = 90; q3min = -90;
% 
% X = zeros(size,1);
% Y = zeros(size,1);
% Z = zeros(size,1);
% 
% for counter = 1:size
%     q1 = deg2rad(rand(1,size)*(q1max-q1min)+q1min);
%     q2 = deg2rad(rand(1,size)*(q2max-q2min)+q2min);
%     q3 = deg2rad(rand(1,size)*(q3max-q3min)+q3min);
%     
%     p0T = DobotForwardKinematics([q1;q2;q3]);
%     X(counter) = p0T(1);
%     Y(counter) = p0T(2);
%     Z(counter) = p0T(3);
% end
% scatter3(X,Y,Z)
% xlabel('x axis')
% ylabel('y axis')
% zlabel('z axis')

% % set robot angles to q by first doing forward, then inverse kinematics
% q = deg2rad(ones(1,3)*45)
% p0T = DobotForwardKinematics(q)
% 
% viable_angles = DobotInverseKinematics(p0T)
% 
% % check that the solutions are valid
% for i = 1:length(viable_angles)
%     q_viable = viable_angles(:,i);
%     assert(norm(p0T - DobotForwardKinematics(q_viable)) < 1e-10)
% end
% 
% % find solution closest in value to previous q
% q_fitted = mod(q+pi,2*pi)-pi;
% min_norm = Inf;
% min_norm_angles = Inf;
% for i = 2:length(viable_angles)
%     if norm(viable_angles(:,i)-q_fitted) < min_norm
%         min_norm = norm(viable_angles(:,i)-q_fitted);
%         min_norm_angles = viable_angles(:,i);
%     end
% end
% 
% SetDobotAngles(min_norm_angles,3)

% % visualize basic angle configuration
% hold on
% q = deg2rad([0 0 0]);
% p_normal = p01 + rot(h1,q(1))*(p12 + rot(h2,q(2))*p23 + rot(h3,q(3))*p3T)
% joints = [[0;0;0] p01 p01+rot(h1,q(1))*(p12) p01+rot(h1,q(1))*(p12+rot(h2,q(2))*p23) p_normal]
% plot(joints(2,:),joints(3,:),'red')
% 
% q = deg2rad([20 0 0]);
% p_joint1 = p01 + rot(h1,q(1))*(p12 + rot(h2,q(2))*p23 + rot(h3,q(3))*p3T)
% joints = [[0;0;0] p01 p01+rot(h1,q(1))*(p12) p01+rot(h1,q(1))*(p12+rot(h2,q(2))*p23) p_joint1]
% plot(joints(2,:),joints(3,:),'blue')
% 
% q = deg2rad([0 20 0]);
% p_joint2 = p01 + rot(h1,q(1))*(p12 + rot(h2,q(2))*p23 + rot(h3,q(3))*p3T)
% joints = [[0;0;0] p01 p01+rot(h1,q(1))*(p12) p01+rot(h1,q(1))*(p12+rot(h2,q(2))*p23) p_joint2]
% plot(joints(2,:),joints(3,:),'green')
% 
% q = deg2rad([0 0 20]);
% p_joint3 = p01 + rot(h1,q(1))*(p12 + rot(h2,q(2))*p23 + rot(h3,q(3))*p3T)
% joints = [[0;0;0] p01 p01+rot(h1,q(1))*(p12) p01+rot(h1,q(1))*(p12+rot(h2,q(2))*p23) p_joint3]
% plot(joints(2,:),joints(3,:),'black')
% hold off