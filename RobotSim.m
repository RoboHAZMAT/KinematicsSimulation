function RoboHAZMAT = RobotSim(options)
clear all; close all; clc;

%% ========================RoboHAZMAT Simulation===========================
% Creates a struct to store all the tobot kinematics
RobotKinematics = struct();

% Define the kinematics that are going to be added to the robot

% Head Kinematics
HK = HeadKinematics;
% HK = RotateKinematicChain(HK,[-pi/24;-pi/6]);
RobotKinematics.HK = HK;

% Left Manupulator Kinematics
LMK = LeftManipulatorKinematics;
%LMK = RotateKinematicChain(LMK,[-pi/3;-pi/4;pi/12;-pi/2.5;0;-pi/6]);
LMK = RotateKinematicChain(LMK,[0;0;0;0;0;0]);

RobotKinematics.LMK = LMK;

% Right Manipulator Kinematics
RMK = RightManipulatorKinematics;
%RMK = RotateKinematicChain(RMK,[-pi/6;pi/4;-pi/12;-pi/3;pi;pi/6]);
RobotKinematics.RMK = RMK;

% Create the RoboHAZMAT Robot object
RoboHAZMAT = Robot(RobotKinematics);

%% =======================Plotting the Simulation==========================
RobotPlot(RoboHAZMAT);

%% ========================Interactive Simulation==========================
if (nargin == 1)
    RoboHAZMAT = InteractiveSim(RoboHAZMAT);
end

fprintf('\nFinished Simulation.\nDone.\n\n');