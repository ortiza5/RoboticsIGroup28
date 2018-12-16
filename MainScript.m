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

% raise dobot to base configuration
angles = SetDobotAngles(dobot,[0;0;0],3);

% set camera extrinsic parameters
POT = DobotForwardKinematics(angles);
POC = [10;0;-50] + POT; % TODO better measure this
ROC = roty(pi/2); % camera points down

RCO = inv(ROC);
PCO = -RCO*POC;
T = [RCO PCO];

% set camera instrinsic parameters
load cameraParams.mat cameraParams
K = cameraParams.IntrinsicMatrix';

% ========================================================================
% TODO - Take image somehow? (replace this line)
% ========================================================================
raw_image = imread('testing.jpg');

% convert it to binary
binary_image = ImageProcessing(raw_image);

while(1)
    % Get Phantom Omni initial position to compare against
    q = callAngles(omni);
    q = q(1:3);
    POT_omniINIT = OmniForwardKinematics(q);
    
    % Pause to allow for movement of the Phantom Omni
    pause(.1)
    
    % Get the new position of the Phantom Omni
    q = callAngles(omni);
    q = q(1:3);
    POT_omniFIN = OmniForwardKinematics(q);
    
    % Find change in Phantom Omni position
    delta_POT = scale*(POT_omniFIN - POT_omniINIT);
    
    % Get new desired position of Dobot
    angles = GetDobotAngles(dobot);
    POT_dobotINIT = DobotForwardKinematics(angles);
    POT_dobotFIN = POT_dobotINIT + delta_POT;
    
    % Convert dobot final position to camera frame
    pi = K*T*[POT_dobotFIN; 1];
    pixelFIN = [pi(1)/pi(3); pi(2)/pi(3)];
    
    % Find closest workspace location (can be the same as current)
    pixelCLOSEST = ClosestWorkspace(pixelFIN, binary_image);
    
    % Convert movement vector to world frame
    piCLOSEST = [pixelCLOSEST*pi(3); pi(3)];
    POT_dobotCLOSEST = [ROC POC]*[K\piCLOSEST; 1];
    
    % move Dobot to closest position
    setPosition(POT_dobotCLOSEST);
    
    % give torque feedback
    feedback_omni = (POT_dobotFIN - POT_dobotCLOSEST)/scale;
    % TODO convert feedback_omni (in mm) to joint torques (using jacobian?)
end



