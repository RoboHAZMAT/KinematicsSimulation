function Robot = SimulationM4(Robot)
%% ===========Keyboard Controlled Mechatronic Arm Simulation===============

% Initializes the arm states and setp size
[RobotFigure, states] = SetupKeyboardControl(Robot);

% Sets up the kinematics for the arm
controlPoint = 6;
KC = Robot.KinematicChains.MAK;
KC.optimization.weightings(controlPoint) = 100;
pointsd = zeros(4, size(KC.points.kP,2));

count = [0;0;0];

% Runs the loop a given number of times
while(states.run)
    % Get current states
    states = guidata(RobotFigure);
    
    % Pull out desired points
    pointsd(:,controlPoint) = [states.location;1];
    
    % Inverse kinematics
    X = InverseKinematicOptimization(KC,pointsd);
    % Rotates the KC
    KC = RotateKinematicChain(KC,X);
    KC = RotateKinematicPoints(KC,X);
    Robot.KinematicChains.MAK = KC;
    
    % Plots the Robot
    RobotPlot(Robot);
    drawnow;
    
    X = X*180/pi;
    fprintf(1, repmat('\b',1,sum(count)));
    count(1) = fprintf('  Base Yaw   Base Pitch   Elbow Pitch   Wrist Pitch\n');
    count(2) = fprintf('%9.3f%12.3f%14.3f%14.3f\n\n',...
        X(1),X(2),X(3),X(4));
    count(3) = fprintf('  X: %.3f           Y: %.3f            Z: %.3f\n',...
        KC.points.kPG(1,controlPoint),KC.points.kPG(2,controlPoint),...
        KC.points.kPG(3,controlPoint));
end