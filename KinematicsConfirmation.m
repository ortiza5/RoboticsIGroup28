clc
close all

% check forward and inverse kinematics give consistent results
% q = [rand()*270-135; rand()*90-5; rand()*100-10]
q = rand(3,1)*360-180
p0T = DobotForwardKinematics(q)

viable_angles = DobotInverseKinematics(p0T)

% check that the solutions are valid
for i = 1:size(viable_angles,2)
    q_viable = viable_angles(:,i);
    assert(norm(p0T - DobotForwardKinematics(q_viable)) < 1e-10)
end

% % visualize the work space
% size = 100;
% q1max = 135; q1min = -135;
% q2max = 85; q2min = -5;
% q3max = 90; q3min = -90;
% 
% X = zeros(size,1);
% Y = zeros(size,1);
% Z = zeros(size,1);
% 
% for i = 1:size
%     q = [rand()*270-135; rand()*90-5; rand()*100-10];
%     
%     p0T = DobotForwardKinematics(q);
%     X(i) = p0T(1);
%     Y(i) = p0T(2);
%     Z(i) = p0T(3);
% end
% scatter3(X,Y,Z)
% xlabel('x axis')
% ylabel('y axis')
% zlabel('z axis')

% % visualize basic angle configuration (not updated)
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