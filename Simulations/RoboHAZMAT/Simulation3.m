function Robot = Simulation3(Robot)
%% ==============Keyboard Controlled Robot Arms Simulation=================

% Sets up the Keyboard Control with Advanced control
[RobotFigure, states] = SetupKeyboardControl(Robot, 1);

% Sets up the kinematics for the Right arm
controlPoint = 6;
KCR = Robot.KinematicChains.RMK;
KCR.optimization.weightings(controlPoint) = 10;
pointsdR = zeros(4, size(KCR.points.kP,2));

% Sets up the kinematics for the Left arm
KCL = Robot.KinematicChains.LMK;
KCL.optimization.weightings(controlPoint) = 10;
pointsdL = zeros(4, size(KCL.points.kP,2));

% Runs the loop a given number of times
while(states.run)
    % Get current states
    states = guidata(RobotFigure);
    
    % Pull out desired points
    pointsdR(:,controlPoint) = [states.location(:,1);1];
    pointsdL(:,controlPoint) = [states.location(:,2);1];
    
    % Inverse kinematics
    XR = InverseKinematicOptimization(KCR,pointsdR);
    
    % Rotates the KC
    KCR = RotateKinematicChain(KCR,XR);
    Robot.KinematicChains.RMK = KCR;
    
    % Inverse kinematics
    XL = InverseKinematicOptimization(KCL,pointsdL);
    
    % Rotates the KC
    KCL = RotateKinematicChain(KCL,XL);
    Robot.KinematicChains.LMK = KCL;
    
    % Plots the Robot
    RobotPlot(Robot);
    drawnow;
end