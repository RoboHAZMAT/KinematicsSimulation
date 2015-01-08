function Robot = SimulationM3(Robot)
%% ===============Mechatronic Arm Controlled Through Arduino===============

% Sets up the Keyboard Control
[RobotFigure, states] = SetupKeyboardControl;

% Initializes communication with Mechatronic Arm
[IMUCOM, motorControlCOM] = SetupCOM;
[serialMotorControl, motor] = ...
    SetupMotorControlSerial(motorControlCOM);
clc;

% Sets up the kinematics
controlPoint = 6;
KC = Robot.KinematicChains.MAK;
KC.optimization.weightings(controlPoint) = 100;
pointsd = zeros(4, size(KC.points.p,2));

% History vectors for trajectories
xHist = zeros(1,360);
yHist = zeros(1,360);
zHist = zeros(1,360);
xHistT = zeros(1,360);
yHistT = zeros(1,360);
zHistT = zeros(1,360);

% Trajectory parameters
traj = 4;
velocity = 1; % Set velocity: {1,..,.5}
trajDur = 1;  % Trajectory segment duration
n = 15;       % 13 for a full cycle
count = zeros(3,1);

% Option parameters
varsin.trajDur = trajDur;
varsin.serialMotorControl = serialMotorControl;
varsin.motor = motor;

% Open gripper to start
serialMotorControl.servoWrite(motor.gripper, 0);
pause(0.5);

while (states.run)
    % Runs the trajectory multiple times
    for i = 1:velocity:(n*trajDur - 1)
        
        % Get current states
        states = guidata(RobotFigure);
        if (~states.run), break; end;
        
        % Create the trajectory
        [x, y, z] = TrajectoriesMechatronicArm(i, traj, varsin);
        
        % Assigns the desired trajectory history
        xHist(i) = x;
        yHist(i) = y;
        zHist(i) = z;
        
        % Adds noise to trajectory and carries out inverse
        % kinematics on the noisy trajectory
        pointsd(:,controlPoint) = [x; y; z; 1];
        X = InverseKinematicOptimization(KC,pointsd);
        
        % Rotates and plots the right arm to optimized value
        KC = RotateKinematicChain(KC,X);
        Robot.KinematicChains.MAK = KC;
        RobotPlot(Robot);
        
        % Plots the ghost trajectories
        xHistT(i) = KC.points.pG(1,5);
        yHistT(i) = KC.points.pG(2,5);
        zHistT(i) = KC.points.pG(3,5);
        
        % plot3(xHist,yHist,zHist,'.','color','green');
        % plot3(xHistT,yHistT,zHistT,'.','color','red');
        drawnow;
        
        % Move arm servos
        MechatronicArmControl(serialMotorControl, motor, X);
        
        X = X*180/pi;
        fprintf(1, repmat('\b',1,sum(count)));
        count(1) = fprintf('  Base Yaw   Base Pitch   Elbow Pitch   Wrist Pitch\n');
        count(2) = fprintf('%9.3f%12.3f%14.3f%14.3f\n\n',...
            X(1),X(2),X(3),X(4));
        count(3) = fprintf('    X: %.3f           Y: %.3f            Z: %.3f\n',...
            KC.points.pG(1,5),KC.points.pG(2,5),KC.points.pG(3,5));
    end
end