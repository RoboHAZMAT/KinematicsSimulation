function [RoboHAZMAT, MechatronicArm] = RobotSim(options)
%% ===========================Robot Simulation=============================
% RoboHAZMAT: Senior Design Project
% Motion Control Team
% Gerardo Bledt
% October 15, 2014
%
% Main running program for the Robot simulations as part of the RoboHAZMAT
%   senior design team. The project consists of an intuitive gesture
%   controlled robotic unit for use in hazardous response situations.
%
% This Kinematics Simulation repo is designed to test the Motion Control
%   algorithms on simulated robots. The eventual RoboHAZMAT robot is
%   simulated by defining the necessary kinematics files and rotating the 6
%   degrees of freedom for each arm and the 3 degrees of freedom for the
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
%   - Optimize speed of the code as much as possible
%   - Add better commenting and documentation
%   - Create instructions file for how to take advantage of all functions
clc; %close all;

fprintf('=======================================\n\n');
fprintf('   RoboHAZMAT: Senior Design Project   \n');
fprintf('             Virginia Tech             \n\n');
fprintf('=======================================\n\n');

%% ========================RoboHAZMAT Simulation===========================
% Creation of the various kinematic chain elements in the RoboHAZMAT robot.
% The Robot object is comprised of a head, left manipulator, right
% manipulator, and a body frame.

RoboHAZMAT = CreateRobotRoboHAZMAT;
fprintf('---------------------------------------\n\n');

%% ======================Mechatronic Arm Simulation========================
% Creation of the various kinematic chain elements in the Mechatronic Arm.
% The Robot object is comprised of the Mechtronic Arm kinematics and base.

MechatronicArm = CreateRobotMechatronicArm;
fprintf('---------------------------------------\n\n');

%% ========================Interactive Simulation==========================
% Interactive Simulation for the Robots. Mode options are 1 - 6 currently.
% Consult the 'InteractiveSim.m' for instructions and guidance.

if (nargin == 0), mode = 1;
else mode = options(1); end;
if (mode - 10 <= 0)
    % Enter the Interactive Simulation with the RoboHAZMAT Robot
    RoboHAZMAT = InteractiveSim(RoboHAZMAT, mode);
    
elseif (mode - 20 <= 0)
    % Enter the Interactive Simulation with the Mechatronic arm
    MechatronicArm = InteractiveSim(MechatronicArm, mode);
end

states = guidata(gcf);
% Ending the Simulation
fprintf('\nFinished Simulation.\nDone.\n\n');
SetSimulationControlText(states,'Quitting Simulation...', '',...
    'Finished Simulation.', 'Done.');
set(states.statusAxes,'Color',[1 0 0]);
set(states.statusText,'String','Simulation Stopped.');