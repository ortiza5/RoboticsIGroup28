clc
clear all

global p01 p12 p23 p3T

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

angles = DobotInverseKinematics([-2;2;0], angles);
DobotForwardKinematics(angles)
SetDobotAngles(angles, 3)

angles = DobotInverseKinematics([-2;-2;0], angles);
DobotForwardKinematics(angles)
SetDobotAngles(angles, 3)

angles = DobotInverseKinematics([2;-2;0], angles);
DobotForwardKinematics(angles)
SetDobotAngles(angles, 3)