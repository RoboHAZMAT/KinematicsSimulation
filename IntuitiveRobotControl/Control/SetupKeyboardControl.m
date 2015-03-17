function [RobotFigure, states] = SetupKeyboardControl(Robot, option)
%% ============================Setup States================================
% RoboHAZMAT: Senior Design Project
% Motion Control Team
% Gerardo Bledt
% January 7, 2014
%
% Sets up the necessary states for the simulations to use the keyboard
% control function and interact with the simulations in real-time.
states = guidata(gcf);

% Choose Advanced or Basic control
if (nargin == 0)
    states.advancedControl = 0;
    states.trajectoryControl = 0;

elseif (option == 1)
    states.advancedControl = 1;
    states.trajectoryControl = 0;
    
elseif (option == 2)
    states.advancedControl = 0;
    states.trajectoryControl = 1;
end

% Define states
if (states.advancedControl)
    if (strcmpi(Robot.Name,'RoboHAZMAT'))
        % States for the RoboHAZMAT Robot
        states.run = 1;
        states.step = 0.01;
        states.max = 0.62;
        states.min = -0.62;
        states.base(:,1) = [0;-0.0179;0.371];
        states.base(:,2) = [0;0.0179;0.371];
        states.start(:,1) = [0;-0.179;-0.241];
        states.start(:,2) = [0;0.179;-0.241];
        states.location(:,1) = [0;-0.179;-0.241];
        states.location(:,2) = [0;0.179;-0.241];
        clc;
        fprintf('       RoboHAZMAT: Advanced Keyboard Controls        \n');
        fprintf('=====================================================\n');
        fprintf('      XR                YR                 ZR        \n');
        fprintf('-----------------------------------------------------\n');
        fprintf('  I: Forward         L: Right         Semicolon: Up  \n');
        fprintf('  K: Backward        J: Left           Slash: Down   \n');
        fprintf('-----------------------------------------------------\n\n');
        fprintf('      XL                YL                 ZL        \n');
        fprintf('-----------------------------------------------------\n');
        fprintf('  A: Forward         F: Right             A: Up      \n');
        fprintf('  D: Backward        S: Left           Shift: Down   \n');
        fprintf('                  {Space: Quit}\n\n');
        
    elseif (strcmpi(Robot.Name,'Mechatronic Arm'))
        % States for the Mechatronic Arm
        states.run = 1;
        states.step = 0.005;
        states.max = 0.2422;
        states.min = 0;
        states.base(:,1) = [0;0;0.0];%61];
        states.start(:,1) = [0.15;0;0.1];
        states.location(:,1) = [0.15;0;0.1];
        states.gripper = 0;
        clc;
        fprintf('     Mechatronic Arm: Advanced Keyboard Controls\n');
        fprintf('=====================================================\n');
        fprintf('       X                 Y                  Z        \n');
        fprintf('-----------------------------------------------------\n');
        fprintf('  A: Forward         F: Right             A: Up      \n');
        fprintf('  D: Backward        S: Left           Shift: Down   \n');
        fprintf('                  {Space: Quit}\n\n');
    end
    
elseif (states.trajectoryControl)
    % States for Trajectory control
    states.run = 1;
    states.recordTraj = 0;
    states.clearTraj = 0;
    
else
    % States for Basic control
    states.run = 1;
    Robot = [];
end

% Gets the current figure
RobotFigure = gcf;

% Initializes the keyboard key interrupt
set(RobotFigure,'windowkeypressfcn',...
    @(RobotFigure,event)KeyboardControl(RobotFigure,event,Robot));

% Set GUI data
guidata(RobotFigure, states);