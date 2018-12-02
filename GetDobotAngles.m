function actual_angles = GetDobotAngles(dobot)
% Sets Dobot angles to desired angles, waits, then outputs the actual
% angles
% INPUTS:
%   dobot - RobotRaconteur.Connect thing
% OUTPUTS:
%   actual_angles - feedback of actual angles of the robot IN DEGREES

actual_angles = dobot.getJointPositions();
actual_angles = double([actual_angles(1) actual_angles(2) actual_angles(3)]);
end