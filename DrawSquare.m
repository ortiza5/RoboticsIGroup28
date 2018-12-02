clc
clear all

dobot = RobotRaconteur.Connect('tcp://localhost:10001/dobotRR/dobotController');

% get Dobot angles when pen is at center position
angles = GetDobotAngles(dobot);

% set offset so that pen starting position is [0;0;0]
% desired = DobotForwardKinematics(angles) + offset
offset = [0;0;0] - DobotForwardKinematics(angles)

desired = [4;0;0];
angles = DobotInverseKinematics(desired-offset,angles);
SetDobotAngles(dobot,angles,3);

desired = [0;0;0];
angles = DobotInverseKinematics(desired-offset,angles);
SetDobotAngles(dobot,angles,3);

desired = [0;4;0];
angles = DobotInverseKinematics(desired-offset,angles);
SetDobotAngles(dobot,angles,3);

desired = [4;4;0];
angles = DobotInverseKinematics(desired-offset,angles);
SetDobotAngles(dobot,angles,3);

desired = [4;0;0];
angles = DobotInverseKinematics(desired-offset,angles);
SetDobotAngles(dobot,angles,3);