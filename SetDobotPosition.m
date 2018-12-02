function actual_angles = SetDobotPosition(dobot, desired_position, delay_seconds)
% Sets Dobot angles to desired angles, waits, then outputs the actual
% angles
% INPUTS:
%   desired_position - xyz desired position IN MM
%   delay_seconds - wait this long before getting joint positions
% OUTPUTS:
%   actual_angles - feedback of actual angles of the robot IN DEGREES

desired_position = int16(desired_position);

% setAbsolutePosition(self, cartesian, p1, p2, p3, p4):
dobot.setAbsolutePosition(TRUE,desired_position(1),desired_position(2),desired_position(3),int16(0));
pause(delay_seconds);

actual_angles = dobot.getJointPositions();
actual_angles = double([actual_angles(1) actual_angles(2) actual_angles(3)]);
end