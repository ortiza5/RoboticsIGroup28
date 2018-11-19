clc
clear all

global p01 p12 p23 p3T robot
robot = RobotRaconteur.Connect('tcp://localhost:10001/dobotRR/dobotController');


p01 = [0;0;0];
p12 = [0;0;0];
p23 = [0;0;13.5];
p3T = [0;16;0];

% get Dobot angles when pen is on paper, to find p01 (center xyz about
% calibration point)
angles = SetDobotAngles([], 0);
p0Tminusp01 = DobotForwardKinematics(angles);
p01 = -p0Tminusp01;

angles = DobotInverseKinematics([2;2;0], angles);
DobotForwardKinematics(angles)
SetDobotAngles(angles, 3)

% angles = DobotInverseKinematics([-2;2;0], angles);
% DobotForwardKinematics(angles)
% SetDobotAngles(angles, 3)
% 
% angles = DobotInverseKinematics([-2;-2;0], angles);
% DobotForwardKinematics(angles)
SetDobotAngles([200,20,110,0,0], 3)
% 
% for i = linspace(-45,45,10)
%     for j = linspace(-45,45,10)
%         for k = linspace(-45,45,10)
%             SetDobotAngles([i j k],2)
%         end
%     end
% end

% angles = DobotInverseKinematics([2;-2;0], angles);
% DobotForwardKinematics(angles)
% SetDobotAngles(angles, 3)