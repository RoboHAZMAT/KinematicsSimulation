function FigureSetup(Robot)
%% ========================Rotate Kinematic Chain==========================
% RoboHAZMAT: Senior Design Project
% Motion Control Team
% Gerardo Bledt
% December 10, 2014
%
% Setup of the simulation figure for nicer plot. Custom to each robot
% because of varying dimentions and ranges of motion. 

if (strcmp(Robot.Name, 'RoboHAZMAT'))
    % Figure setup for RobotHAZMAT robot
    set(gcf,'Name','RoboHAZMAT Simulation');
    pbaspect([1 1 1]);
    set(gca, 'color', [1 1 1]); set(gcf, 'color', [1 1 1]);
    FS = 18;xlabel('X [m]','FontSize',FS);
    title('RoboHAZMAT Simulation','FontSize',FS);
    ylabel('Y [m]','FontSize',FS);zlabel('Z [m]','FontSize',FS);
    axis([-0.4 0.8 -0.6 0.6 -0.4 0.8]);
    hold on; grid on;
    
elseif (strcmp(Robot.Name, 'Mechatronic Arm'))
    % Figure setup for Mechatronic Arm
    set(gcf,'Name','Mechatronic Arm Simulation');
    pbaspect([1 1 3/5]);
    set(gca, 'color', [1 1 1]); set(gcf, 'color', [1 1 1]);
    FS = 18;xlabel('X [m]','FontSize',FS);
    title('Mechtronic Arm Simulation','FontSize',FS);
    ylabel('Y [m]','FontSize',FS);zlabel('Z [m]','FontSize',FS);
    axis([-0.25 0.25 -0.25 0.25 0.0 0.30]);
    hold on; grid on;
    
else
    % Figure setup for generic robot
    set(gcf,'Name','Robot Simulation');
    pbaspect([1 1 1]);
    set(gca, 'color', [1 1 1]); set(gcf, 'color', [1 1 1]);
    FS = 18;xlabel('X [m]','FontSize',FS);
    title('Robot Simulation','FontSize',FS);
    ylabel('Y [m]','FontSize',FS);zlabel('Z [m]','FontSize',FS);
    axis([-1 1 -1 1 -1 1]);
    hold on; grid on;
end