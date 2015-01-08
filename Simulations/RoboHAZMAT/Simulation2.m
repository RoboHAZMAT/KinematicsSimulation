function Robot = Simulation2(Robot)
%% ================Inverse Kinematic Tracing Trajectories==================

% Sets up the Keyboard Control
[RobotFigure, states] = SetupKeyboardControl;

% Sets up the kinematics
KC = Robot.KinematicChains.RMK;
KC.optimization.weightings(6) = 1;
pointsd = zeros(4, size(KC.points.kP,2));

% Trajectory parameters
traj = 1;      % {1, 2}
velocity = 8;  % Set velocity
trajDur = 60;  % Trajectory segment duration
n = 6;         % 6 for a full 360 cycle
runs = (n*trajDur - 1); % Number of runs

% History vectors for the trajectories
xHist = zeros(1,runs);
yHist = zeros(1,runs);
zHist = zeros(1,runs);
xHistT = zeros(1,runs);
yHistT = zeros(1,runs);
zHistT = zeros(1,runs);

while (states.run)
    % Runs the loop a given number of times
    for i = 1:velocity:runs
        
        % Create the trajectory
        [x, y, z] = TrajectoriesRoboHAZMAT(i, traj);
        
        % Assigns the desired trajectory history
        xHist(i) = x;
        yHist(i) = y;
        zHist(i) = z;
        
        % Inverse kinematics (Noise can be added or removed)
        pointsd(:,size(KC.points.p,2)) = [x;y;z;1] ...
            + [NoiseCalc;NoiseCalc;NoiseCalc;1];
        X = InverseKinematicOptimization(KC,pointsd);
        
        % Rotates and plots the Robot object
        KC = RotateKinematicChain(KC,X);
        Robot.KinematicChains.RMK = KC;
        RobotPlot(Robot);
        
        % Saves the ghost trajectories
        xHistT(i) = KC.points.pG(1,6);
        yHistT(i) = KC.points.pG(2,6);
        zHistT(i) = KC.points.pG(3,6);
        
        % Plots both trajectories
        plot3(xHist,yHist,zHist,'.','color','green');
        plot3(xHistT,yHistT,zHistT,'.','color','red');
        drawnow;
        
        % Get current states
        states = guidata(RobotFigure);
        if (~states.run), break; end;
    end
end