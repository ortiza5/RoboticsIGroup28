clc
close all
clear all

global offset scale dobot omni

% Factor to scale the workspace of the Phantom Omni to the Dobot
scale = 1;

% Initialize the Dobot and Phantom Omni
dobot = RobotRaconteur.Connect('tcp://localhost:10001/dobotRR/dobotController');
omni = RobotRaconteur.Connect('tcp://127.0.0.1:5150/PhantomOmniSimulinkHost/PhantomOmni');

% get Dobot angles when pen is at center position
angles = GetDobotAngles(dobot);
% angles = [0;45;45];

% set offset so that pen starting position is [0;0;0]
% desired = DobotForwardKinematics(angles) + offset
offset = [0;0;0] - DobotForwardKinematics(angles);

% ========================================================================
% TODO - Raise Dobot to get a picture of the paper and setup boundary
% ========================================================================

% take image from camera and set it to raw_image
raw_image = imread('testing.jpg');
binary_image = ImageProcessing(raw_image);

while(1)
    % Get Phantom Omni initial position to compare against
    q = omni.ActualJointAngles
    q = q(1:3);
    P0T_omniINIT = OmniForwardKinematics(q)
    
    % Pause to allow for movement of the Phantom Omni
    pause(.1)
    
    % Get the new position of the Phantom Omni
    q = omni.ActualJointAngles
    q = q(1:3);
    P0T_omniFIN = OmniForwardKinematics(q)
    
    % Find change in Phantom Omni position
    delta_P0T = scale*(P0T_omniFIN - P0T_omniINIT)
    
    % ========================================================================
    % TODO - Send torque to phantom omni if dobot will go outside the boundary
    % ========================================================================
    
    % Send new position to Dobot
    angles = GetDobotAngles(dobot)
    P0T_dobotINIT = DobotForwardKinematics(angles);
    P0T_dobotFIN = P0T_dobotINIT + delta_P0T;
    setPosition(P0T_dobotFIN);
end