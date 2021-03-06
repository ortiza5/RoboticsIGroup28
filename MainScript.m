clc
close all
clear all

global scale dobot omni

% Factor to scale the workspace of the Phantom Omni to the Dobot
scale = 1.5;

% Initialize the Dobot and Phantom Omni
omni = RobotRaconteur.Connect('tcp://127.0.0.1:5150/PhantomOmniSimulinkHost/PhantomOmni')
dobot = RobotRaconteur.Connect('tcp://localhost:10001/dobotRR/dobotController')

rotz = [cos(-pi/2) -sin(-pi/2) 0; sin(-pi/2) cos(-pi/2) 0; 0 0 1];

q = omni.ActualJointAngles;
q = q(1:3);
P0T_omniORIGINAL = rotz*OmniForwardKinematics(q)

angles = [0;0;0];
POT_dobotORIGINAL = DobotForwardKinematics(angles);

% ========================================================================
% TODO - Raise Dobot to get a picture of the paper and setup boundary
% ========================================================================

while(1)
%     % Get Phantom Omni initial position to compare against
%     q = omni.ActualJointAngles;
%     q = q(1:3);
%     P0T_omniINIT = OmniForwardKinematics(q);
%     
%     % Pause to allow for movement of the Phantom Omni
    pause(0.5)
    
    % Get the new position of the Phantom Omni
    q = omni.ActualJointAngles;
    q = q(1:3);
    P0T_omniFIN = rotz*OmniForwardKinematics(q);
    
    % ========================================================================
    % TODO - Send torque to phantom omni if dobot will go outside the boundary
    % ========================================================================
    
    POT_dobotFIN = scale*(P0T_omniFIN-P0T_omniORIGINAL) + POT_dobotORIGINAL;
    POT_dobotFIN(3) = 0;
    
    % Send new position to Dobot
    angles = GetDobotAngles(dobot)';
    P0T_dobotINIT = DobotForwardKinematics(angles);
    setPosition(POT_dobotFIN,P0T_dobotINIT)
end