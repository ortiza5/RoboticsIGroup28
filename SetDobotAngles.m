function actual_angles = SetDobotAngles(dobot, desired_angles, delay_seconds)
% Sets Dobot angles to desired angles, waits, then outputs the actual
% angles
% INPUTS:
%   desired_angles - 3 angles to set the joint angles to IN DEGREES
%   delay_seconds - wait this long before getting joint positions
% OUTPUTS:
%   actual_angles - feedback of actual angles of the robot IN DEGREES

desired_angles = int16(desired_angles);

dobot.setJointPositions(desired_angles(1),desired_angles(2),desired_angles(3),int16(0),int16(0));
pause(delay_seconds);

actual_angles = dobot.getJointPositions();
actual_angles = double([actual_angles(1) actual_angles(2) actual_angles(3)]);
end