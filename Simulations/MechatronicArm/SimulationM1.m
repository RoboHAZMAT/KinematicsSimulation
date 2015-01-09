function Robot = SimulationM1(Robot)
%% ========Mechatronic Arm Inverse Kinematics Trajectory Simulation========

% Sets up the Keyboard Control with Advanced control
[RobotFigure, states] = SetupKeyboardControl;

% Sets up the kinematics
controlPoint = 5;
KC = Robot.KinematicChains.MAK;
KC.optimization.weightings(controlPoint) = 100;
pointsd = zeros(4, size(KC.points.kP,2));

% Trajectory parameters
traj.traj = 2;      % {1, 2}
traj = TrajectoriesMechatronicArm(0, traj);

% History vectors for the desired trajectories
histD = zeros(traj.runs,3);

% History vectors for the actual trajectories
histT = zeros(traj.runs,3);

while (states.run)
    % Runs the loop a given number of times
    for i = 1:traj.velocity:traj.runs
        
        % Get current states
        states = guidata(RobotFigure);
        if (~states.run), break; end;
        
        % Create the trajectory
        traj = TrajectoriesMechatronicArm(i, traj);
        
        % Assigns the desired trajectory history
        histD(i,:) = traj.point';
        
        % Adds noise to trajectory and carries out inverse
        % kinematics on the noisy trajectory
        pointsd(:,controlPoint) = [traj.point; 1];
        X = InverseKinematicOptimization(KC,pointsd);
        
        % Rotates and plots the right arm to optimized value
        KC = RotateKinematicChain(KC,X);
        Robot.KinematicChains.MAK = KC;
        RobotPlot(Robot);
        
        % Plots the ghost trajectories
        histT(i,:) = KC.points.kPG(1:3,controlPoint)';
        plot3(histD(:,1),histD(:,2),histD(:,3),'.','color','green');
        plot3(histT(:,1),histT(:,2),histT(:,3),'.','color','red');
        drawnow;
    end
end