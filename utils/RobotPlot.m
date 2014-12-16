function RobotPlot(Robot)
%% ==============================Robot Plot================================
% RoboHAZMAT: Senior Design Project
% Motion Control Team
% Gerardo Bledt
% October 20, 2014
%
% Plots the given Robot object in the figure. Adds a base object if
% necessary and plots the kinematic chains that make up the robot.

% Clears the figure
cla;
LW = 6; % line width
MS = 20; % marker size

%% ===========================Plotting of Base=============================
fields = fieldnames(Robot.KinematicChains);

% Add another elseif statement if another named robot will be used with
% a stationary base structure.

% Sets up the base frame of the RoboHAZMAT Robot
if (strcmp(Robot.Name, 'RoboHAZMAT'))
    % Plotting the Frame Body
    plot3([0,0,0],[-.179,0,.179],[.371,.371,.371],'LineWidth',LW,...
        'Color',[0 0 0]);
    hold on; grid on;
    plot3([0,0,0],[0,0,0],[0,0.371,0.589],'LineWidth',LW,'Color',[0 0 0],...
        'Marker','.','MarkerEdgeColor',0.5*[1 1 1],'MarkerSize',MS);
    hold on; grid on;
    
% Sets up the base frame of the Mechatronic Arm
elseif (strcmp(Robot.Name, 'Mechatronic Arm'))
    % Plotting the Base
    plot3([0,0],[0,0],[0,0.06],'LineWidth',LW,...
        'Color',[0 0 0]);
    hold on; grid on;
    plot3([0,0],[0,0],[0,0.05],'LineWidth',LW,'Color',[0 0 0],...
        'Marker','.','MarkerEdgeColor',0.5*[1 1 1],'MarkerSize',MS);
    hold on; grid on;
end

% Uncomment next line to plot a square robot base (glitchy)
% RobotBody;

%% ===================Plotting of all Kinematic Chains=====================
% Plots the Kinematic chains that make up the robot
for i = 1:length(fields)
    plot3(Robot.KinematicChains.(fields{i}).points.pG(1,:),...
        Robot.KinematicChains.(fields{i}).points.pG(2,:),...
        Robot.KinematicChains.(fields{i}).points.pG(3,:),...
        'LineWidth',LW,'Color',[0 0 0],'Marker','.',...
        'MarkerEdgeColor',[1 0 0],'MarkerSize',MS);
    hold on; grid on;
end