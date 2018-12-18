function setPosition(desired,previous)
global dobot
angles = DobotInverseKinematics(desired,previous)
SetDobotAngles(dobot,angles,0);
end