function Robot = SimulationM4(Robot)
%% ===========Keyboard Controlled Mechatronic Arm Simulation===============

% Initializes the arm states and setp size
[RobotFigure, states] = SetupKeyboardControl(Robot, 1);

% Sets up the kinematics for the arm
controlPoint = 6;
KC = Robot.KinematicChains.MAK;
KC.optimization.weightings(controlPoint) = 100;
pointsd = zeros(4, size(KC.points.kP,2));

% Status report options
status.count = 0;
status.point = [1;2;3;4;5;6];

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
    
    % Prints the angle and point status report of the KC
    status = PrintStatusReport(KC, X, status);
end