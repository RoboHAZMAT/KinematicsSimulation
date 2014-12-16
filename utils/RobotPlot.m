function RobotPlot(Robot)
% Plots a given Robot object

% Clears the figure
cla;
LW = 6; % line width
MS = 20; % marker size

fields = fieldnames(Robot.KinematicChains);

% Sets up the figure for plotting the RoboHAZMAT Robot
if (strcmp(Robot.Name, 'RoboHAZMAT'))
    % Plotting the Frame Body
    plot3([0,0,0],[-.179,0,.179],[.371,.371,.371],'LineWidth',LW,...
        'Color',[0 0 0]);
    hold on; grid on;
    plot3([0,0,0],[0,0,0],[0,0.371,0.589],'LineWidth',LW,'Color',[0 0 0],...
        'Marker','.','MarkerEdgeColor',0.5*[1 1 1],'MarkerSize',MS);
    hold on; grid on;
    
% Sets up the figure for plotting the Mechatronic Arm
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

% Plots the Kinematic chains that make up the robot
for i = 1:length(fields)
    plot3(Robot.KinematicChains.(fields{i}).points.pG(1,:),...
        Robot.KinematicChains.(fields{i}).points.pG(2,:),...
        Robot.KinematicChains.(fields{i}).points.pG(3,:),...
        'LineWidth',LW,'Color',[0 0 0],'Marker','.',...
        'MarkerEdgeColor',[1 0 0],'MarkerSize',MS);
    hold on; grid on;
end