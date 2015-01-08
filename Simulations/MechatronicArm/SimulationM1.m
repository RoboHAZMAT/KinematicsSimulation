function Robot = SimulationM1(Robot)
%% ========Mechatronic Arm Inverse Kinematics Trajectory Simulation========

% Sets up the Keyboard Control with Advanced control
[RobotFigure, states] = SetupKeyboardControl;

% Sets up the kinematics
KC = Robot.KinematicChains.MAK;
KC.optimization.weightings(5) = 100;
pointsd = zeros(4, size(KC.points.p,2));

% Trajectory parameters
traj = 1;      % {1, 2, 3}
velocity = 5;  % Set velocity
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
        [x, y, z] = TrajectoriesMechatronicArm(i, traj);
        
        % Assigns the desired trajectory history
        xHist(i) = x;
        yHist(i) = y;
        zHist(i) = z;
        
        % Adds noise to trajectory and carries out inverse
        % kinematics on the noisy trajectory
        pointsd(:,size(KC.points.p,2)) = [x; ...
            y; z;1];
        X = InverseKinematicOptimization(KC,pointsd);
        
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
        
        % Get current states
        states = guidata(RobotFigure);
        if (~states.run), break; end;
    end
end