clc
close all
clear all

global offset dobot

dobot = RobotRaconteur.Connect('tcp://localhost:10001/dobotRR/dobotController');

% get Dobot angles when pen is at center position
angles = GetDobotAngles(dobot);
% angles = [0;45;45];

% set offset so that pen starting position is [0;0;0]
% desired = DobotForwardKinematics(angles) + offset
offset = [0;0;0] - DobotForwardKinematics(angles)

% % make 4 inches x 4 inches square (4 inches = 50 mm)
% setPosition([-50;-50;0])
% setPosition([-50; 50;0])
% setPosition([ 50; 50;0])
% setPosition([ 50;-50;0])
% setPosition([-50;-50;0])
% % set back to original point
% setPosition([  0;  0;0])

for theta = linspace(0,360,20)
    setPosition(50*cosd(theta),50*sind(theta))
end

function setPosition(desired)
global offset dobot
angles = DobotInverseKinematics(desired-offset)
SetDobotAngles(dobot,angles,0.5);
end

% positions = []
% setPosition([-20;-20;-20])
% setPosition([-20; 20;-20])
% setPosition([ 20; 20;-20])
% setPosition([ 20;-20;-20])
% setPosition([-20;-20;-20])
% setPosition([-20;-20; 20])
% setPosition([-20; 20; 20])
% setPosition([ 20; 20; 20])
% setPosition([ 20;-20; 20])
% setPosition([-20;-20; 20])
% 
% positions
% plot3(positions(1,:), positions(2,:), positions(3,:))
% 
% function setPosition(desired)
% global offset positions
% positionFromBaseInches = (desired-offset)/25.4;
% positions = [positions positionFromBaseInches];
% angles = DobotInverseKinematics(desired-offset)
% % SetDobotAngles(dobot,angles,3);
% end