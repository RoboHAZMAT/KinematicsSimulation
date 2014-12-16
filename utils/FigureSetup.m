function FigureSetup(Robot)
% Setup of the simulation figure for nicer plot

if (strcmp(Robot.Name, 'RoboHAZMAT'))
    set(gcf,'Name','Robot Simulation');
    pbaspect([1 1 1]);
    set(gca, 'color', [1 1 1]); set(gcf, 'color', [1 1 1]);
    FS = 18;xlabel('X [m]','FontSize',FS);
    title('RoboHAZMAT Simulation','FontSize',FS);
    ylabel('Y [m]','FontSize',FS);zlabel('Z [m]','FontSize',FS);
    axis([-0.4 0.8 -0.6 0.6 -0.4 0.8]);
elseif (strcmp(Robot.Name, 'Mechatronic Arm'))
    set(gcf,'Name','Mechatronic Arm Simulation');
    pbaspect([1 1 1]);
    set(gca, 'color', [1 1 1]); set(gcf, 'color', [1 1 1]);
    FS = 18;xlabel('X [m]','FontSize',FS);
    title('Mechtronic Arm Simulation','FontSize',FS);
    ylabel('Y [m]','FontSize',FS);zlabel('Z [m]','FontSize',FS);
    axis([-0.3 0.3 -0.3 0.3 -0.1 0.5]);
end