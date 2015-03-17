%% ==========================Addpath RoboHAZMAT============================
% RoboHAZMAT: Senior Design Project
% Motion Control Team
% Gerardo Bledt
% January 15, 2014
%
% Adds the necessary subdirectories to carry out the simulations.

%% Adds the directory containing all of Robot Kinematics
addpath RobotKinematics\
addpath RobotKinematics\RoboHAZMAT
addpath RobotKinematics\MechatronicArm

%% Adds the directory containing the Trajectory Planning and Optimization
addpath TrajectoryPlanningOptimization\
addpath TrajectoryPlanningOptimization\TrajectoryLibrary
addpath TrajectoryPlanningOptimization\Optimization

%% Adds all of the necessary utilities for the simulaitons
addpath Utils\
addpath Utils\Rotation
addpath Utils\Plotting
addpath Utils\Misc

%% Adds the Serial communication scripts
addpath SerialCommunication\
addpath SerialCommunication\IMU
addpath SerialCommunication\MotorControl

%% Adds the Intuitive Control scripts
addpath IntuitiveRobotControl\
addpath IntuitiveRobotControl\Control
addpath IntuitiveRobotControl\StateEstimation

%% Adds the scripts containing the Simulations
addpath Simulations\
addpath Simulations\RoboHAZMAT
addpath Simulations\MechatronicArm