function actual_angles = SetDobotAngles(desired_angles, delay_seconds)
% Sets Dobot angles to desired angles, waits, then outputs the actual
% angles
% INPUTS:
%   desired_angles - 3 angles to set the joint angles to
%   delay_seconds - wait this long before getting joint positions
% OUTPUTS:
%   actual_angles - feedback of actual angles of the robot

robot = RobotRaconteur.Connect('tcp://localhost:10001/dobotRR/dobotController');
desired_angles = int16(rad2deg(desired_angles));
if ~isempty(desired_angles)
    robot.setJointPositions(desired_angles(1),desired_angles(2),desired_angles(3),0);
    pause(delay_seconds);
end
actual_angles = robot.getJointPositions();
actual_angles = deg2rad([actual_angles(1) actual_angles(2) actual_angles(3)]);
end