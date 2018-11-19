robot = RobotRaconteur.Connect('tcp://localhost:10001/dobotRR/dobotController');

%% Gripper
% Read/Write Joint angles
robot.getJointPositions()
robot.setJointPositions(int16(30),int16(0),int16(0),int16(0), int16(0)); % int16 q1, int16 q2, int16 q3, int16 q4, int16 grip
pause(3);

% Read/Write Catesian position
robot.getPositions ()
robot.send_absolute_position(int16(200),int16(20),int16(110),int16(0), int16(0)); % int16 x, int16 y, int16 z, int16 rot, int16 grip
pause(3);
