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

%% ===========================Plotting of Base=============================
fields = fieldnames(Robot.KinematicChains);

% Add another elseif statement if another named robot will be used with
% a stationary base structure.

% Sets up the base frame of the RoboHAZMAT Robot
if (strcmp(Robot.Name, 'RoboHAZMAT'))
    LW = 8; % line width
    MS = 25; % marker size
    % Plotting the Frame Body
    plot3([0,0,0],[-.179,0,.179],[.371,.371,.371],'LineWidth',LW,...
        'Color',[0 0 0]);
    plot3([0,0,0],[0,0,0],[0,0.371,0.589],'LineWidth',LW,'Color',[0 0 0],...
        'Marker','.','MarkerEdgeColor',0.5*[1 1 1],'MarkerSize',MS);
    
    % Sets up the base frame of the Mechatronic Arm
elseif (strcmp(Robot.Name, 'Mechatronic Arm'))
    LW = 15; % line width
    MS = 30; % marker size
    
    % Plotting the Base filled in cylinder
    r = 0.04;
    h = 0.0585;
    color = 0.2;
    [x,y,z] = cylinder(r);
    z(2, :) = h;
    surf(x,y,z, 'FaceColor', color*[1,1,1]);
    theta = 0:0.15*pi:2*pi;
    x = r*cos(theta);
    y = r*sin(theta);
    patch(x,y,h*ones(size(x)), color*[1,1,1]);
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
end