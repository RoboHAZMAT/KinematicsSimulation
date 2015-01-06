function Robot = Simulation2(Robot)
%% ================Inverse Kinematic Tracing Trajectories==================

% Sets up the kinematics
KC = Robot.KinematicChains.RMK;
KC.optimization.weightings(6) = 1;
pointsd = zeros(4, size(KC.points.kP,2));

% History vectors for the trajectories
xHist = zeros(1,360);
yHist = zeros(1,360);
zHist = zeros(1,360);
xHistT = zeros(1,360);
yHistT = zeros(1,360);
zHistT = zeros(1,360);

% Trajectory parameters
traj = 1;      % {1, 2}
velocity = 8;  % Set velocity
runs = 3;      % Number of runs
trajDur = 60;  % Trajectory segment duration
n = 6;         % 6 for a full 360 cycle

% Runs the loop a given number of times
for i = 1:velocity:(n*runs*trajDur - 1)
    % Create the trajectory
    [x, y, z] = TrajectoriesRoboHAZMAT(i, traj);
    
    % Assigns the desired trajectory history
    xHist(i) = x;
    yHist(i) = y;
    zHist(i) = z;
    
    % Inverse kinematics (Noise can be added or removed)
    pointsd(:,size(KC.points.p,2)) = [x;y;z;1] ...
        + [NoiseCalc;NoiseCalc;NoiseCalc;1];
    X = InverseKinematicOptimization(Robot,'RMK',pointsd);
    
    % Rotates and plots the Robot object
    KC = RotateKinematicChain(KC,X);
    Robot.KinematicChains.RMK = KC;
    RobotPlot(Robot);
    
    % Plots the ghost trajectories
    xHistT(i) = KC.points.pG(1,6);
    yHistT(i) = KC.points.pG(2,6);
    zHistT(i) = KC.points.pG(3,6);
    
    plot3(xHist,yHist,zHist,'.','color','green');
    plot3(xHistT,yHistT,zHistT,'.','color','red');
    drawnow;
end