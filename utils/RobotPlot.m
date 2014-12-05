function RobotPlot(Robot)

% Clears the figure
clf;
LW = 6; % line width
MS = 20; % marker size

fields = fieldnames(Robot.KinematicChains);

% Plotting the Frame Body
plot3([0,0,0],[-.179,0,.179],[.371,.371,.371],'LineWidth',LW,...
    'Color',[0 0 0]);
hold on; grid on;
plot3([0,0,0],[0,0,0],[0,0.371,0.589],'LineWidth',LW,'Color',[0 0 0],...
    'Marker','.','MarkerEdgeColor',0.5*[1 1 1],'MarkerSize',MS);
hold on; grid on;

% RobotBody;

for i = 1:length(fields)
    plot3(Robot.KinematicChains.(fields{i}).points.pG(1,:),...
        Robot.KinematicChains.(fields{i}).points.pG(2,:),...
        Robot.KinematicChains.(fields{i}).points.pG(3,:),...
        'LineWidth',LW,'Color',[0 0 0],'Marker','.',...
        'MarkerEdgeColor',[1 0 0],'MarkerSize',MS);
    hold on; grid on;
end

% Figure parameters to format the plot as needed
set(gcf,'Name','Robot Simulation');
pbaspect([1 1 1]);
set(gca, 'color', [1 1 1]); set(gcf, 'color', [1 1 1]);
FS = 18;xlabel('X [m]','FontSize',FS);
title('RoboHAZMAT Simulation','FontSize',FS);
ylabel('Y [m]','FontSize',FS);zlabel('Z [m]','FontSize',FS);
axis([-0.4 0.8 -0.6 0.6 -0.4 0.8]);