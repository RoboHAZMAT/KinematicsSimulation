function [RoboHAZMAT, MechatronicArm] = RobotSim(options)
%% ===========================Robot Simulation=============================
% RoboHAZMAT: Senior Design Project
% Motion Control Team
% Gerardo Bledt
% December 15, 2014
%
% Main running program for the Robot simulations as part of the RoboHAZMAT
%   senior design team. The project consists of an intuitive gesture
%   controlled robotic unit for use in hazardous response situations. 
%
% This Kinematics Simulation repo is designed to test the Motion Control
%   algorithms on simulated robots. The eventual RoboHAZMAT robot is
%   simulated by defining the necessary kinematics files and rotatint the 6
%   degrees of freedoem for each arm and the 3 degrees of freedom for the
%   head. It also allows for Serial communication testing with a prototype 
%   Mechatronic arm 
%
% Currently able to:
%   - Successfully run real-time inverse kinematics
%   - Trajectory tracking of points using optimization of joint angles for
%       the kinematic chains 
%   - Communicate with Arduino over Serial
%   - Control a 6 DOF Mechatronic Arm with intuitive gestures interpreted 
%       by an IMU running sensor fusion algorithms
%   - Control the simulated humanoid robot kinematic chains with the use of
%       the incoming IMU output
%   - Designed for easy expandability and additions
%
% Still to do:
%   - Add functional quaternion rotations of vectors
%   - Multiple IMU communication for full human arm vector representation
%   - Inverse kinematics using IMU sensor estimated human arm positioning
%   - Optimize speed of the code as much as possible
%   - Add better commenting and documentation
%   - Create instructions file for how to take advantage of all functions

close all; clc;

%% ========================RoboHAZMAT Simulation===========================
% Creation of the various kinematic chain elements in the RoboHAZMAT robot.
% The Robot object is comprised of a head, left manipulator, right
% manipulator, and a body frame.

% Creates a struct to store all the robot kinematics
RobotKinematics = struct();

% Define the kinematics that are going to be added to the robot

% Head Kinematics
HK = HeadKinematics;
%HK = RotateKinematicChain(HK,[-pi/24;-pi/6]);
RobotKinematics.HK = HK;

% Left Manupulator Kinematics
LMK = LeftManipulatorKinematics;
%LMK = RotateKinematicChain(LMK,[-pi/4;-pi/6;pi/12;-pi/3;0;-pi/3]);
RobotKinematics.LMK = LMK;

% Right Manipulator Kinematics
RMK = RightManipulatorKinematics;
%RMK = RotateKinematicChain(RMK,[-pi/6;pi/4;-pi/12;-pi/3;pi;pi/6]);
RobotKinematics.RMK = RMK;

% Create the RoboHAZMAT Robot object
RKStruct = struct();
RKStruct.name = 'RoboHAZMAT';
RKStruct.KC = RobotKinematics;
RoboHAZMAT = Robot(RKStruct);


%% ======================Mechatronic Arm Simulation========================
% Creation of the various kinematic chain elements in the Mechatronic Arm.
% The Robot object is comprised of the Mechtronic Arm kinematics and base.

% Mechatronic Arm Kinematics
MAKinematics = struct();
MAK = MechatronicArmKinematics;
MAK = RotateKinematicChain(MAK,[pi/12;pi/4;pi/3;pi/6;0]);
MAKinematics.MAK = MAK;

% Mechatronic Arm Robot
MAKStruct = struct();
MAKStruct.name = 'Mechatronic Arm';
MAKStruct.KC = MAKinematics;
MechatronicArm = Robot(MAKStruct);

%% =======================Plotting the Simulation==========================
% Plots the RoboHAZMAT simulation as a default using the 'RobotPlot.m' and
% the 'FigureSetup.m' utils with the Robot object.

% Run a one time figure setup
FigureSetup(RoboHAZMAT);
% Plot the robot
RobotPlot(RoboHAZMAT);

%% ========================Interactive Simulation==========================
% Interactive Simulation for the Robots. Mode options are 1 - 6 currently.
% Consult the 'InteractiveSim.m' for instructions and guidance.

if (nargin == 1)
    mode = options(1);
    if (mode - 5 <= 0 || mode == 7)
        % Enter the Interactive Simulation with the RoboHAZMAT Robot
        RoboHAZMAT = InteractiveSim(RoboHAZMAT, mode);
    else
        % Run a one time figure setup
        FigureSetup(MechatronicArm);
        % Plot the arm
        RobotPlot(MechatronicArm);
        
        % Enter the Interactive Simulation with the Mechatronic arm
        RoboHAZMAT = InteractiveSim(MechatronicArm, mode);
    end
end

% Ending the Simulation
fprintf('\nFinished Simulation.\nDone.\n\n');