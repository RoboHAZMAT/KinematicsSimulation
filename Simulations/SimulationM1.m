function Robot = SimulationM1(Robot)

% Sets up the kinematics
KC = Robot.KinematicChains.MAK;
KC.optimization.weightings(5) = 100;
pointsd = zeros(4, size(KC.points.p,2));

% History vectors for trajectories
xHist = zeros(1,360);
yHist = zeros(1,360);
zHist = zeros(1,360);
xHistT = zeros(1,360);
yHistT = zeros(1,360);
zHistT = zeros(1,360);

% Trajectory parameters
traj = 2;
velocity = 5; % Set velocity: {1,..,.5}
runs = 3;     % Number of runs
trajDur = 60; % Trajectory segment duration
n = 6;        % 6 for a full 360 cycle

% Runs the trajectory multiple times
for i = 1:velocity:(n*runs*trajDur - 1)
    
    % Create the trajectory
    [x, y, z] = TrajectoriesMechatronicArm(i, traj);
    
    % Assigns the desired trajectory history
    xHist(i) = x;
    yHist(i) = y;
    zHist(i) = z;
    
    % Adds noise to trajectory and carries out inverse
    % kinematics on the noisy trajectory
    pointsd(:,size(KC.points.p,2)) = [x; ...
        y; z;1];
    X = InverseKinematicOptimization(Robot,'MAK',pointsd);
    
    % Rotates and plots the right arm to optimized value
    KC = RotateKinematicChain(KC,X);
    Robot.KinematicChains.MAK = KC;
    RobotPlot(Robot);
    
    % Plots the ghost trajectories
    xHistT(i) = KC.points.pG(1,5);
    yHistT(i) = KC.points.pG(2,5);
    zHistT(i) = KC.points.pG(3,5);
    
    plot3(xHist,yHist,zHist,'.','color','green');
    plot3(xHistT,yHistT,zHistT,'.','color','red');
    drawnow;
end