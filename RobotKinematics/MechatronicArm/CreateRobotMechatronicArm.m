function MechatronicArm = CreateRobotMechatronicArm

fprintf('MECHATRONIC ARM ROBOT\n\n');

% Mechatronic Arm Kinematics
MAKinematics = struct();
MAKinematics.MAK = MechatronicArmKinematics;

% Mechatronic Arm Robot
MAKStruct = struct();
MAKStruct.name = 'Mechatronic Arm';
MAKStruct.KC = MAKinematics;
MechatronicArm = Robot(MAKStruct);
