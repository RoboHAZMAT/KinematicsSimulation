function Robot = SimulationM4(Robot)
%% =================Keyboard Controlled Mechatronic Arm====================

% Initializes the arm states and setp size
[RobotFigure, states] = SetupKeyboardControl(Robot);

% Sets up the kinematics for the arm
controlPoint = 5;
KC = Robot.KinematicChains.MAK;
KC.optimization.weightings(controlPoint) = 100;
pointsd = zeros(4, size(KC.points.kP,2));

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
    Robot.KinematicChains.MAK = KC;
    
    % Plots the Robot
    RobotPlot(Robot);
    drawnow;
end