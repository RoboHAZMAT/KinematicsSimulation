function RoboHAZMAT = CreateRobotRoboHAZMAT

fprintf('ROBOHAZMAT ROBOT\n\n');

% Creates a struct to store all the robot kinematics
RobotKinematics = struct();

% Define the kinematics that are going to be added to the robot

% Head Kinematics
RobotKinematics.HK = HeadKinematics;

% Left Manupulator Kinematics
RobotKinematics.LMK = LeftManipulatorKinematics;

% Right Manipulator Kinematics
RobotKinematics.RMK = RightManipulatorKinematics;

% Create the RoboHAZMAT Robot object
RKStruct = struct();
RKStruct.name = 'RoboHAZMAT';
RKStruct.KC = RobotKinematics;
RoboHAZMAT = Robot(RKStruct);