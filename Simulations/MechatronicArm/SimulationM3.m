function Robot = SimulationM3(Robot)
%% ===============Mechatronic Arm Controlled Through Arduino===============

% Sets up the Keyboard Control
[RobotFigure, states] = SetupKeyboardControl;

% Initializes communication with Mechatronic Arm
[~, motorControlCOM] = SetupCOM;
[serialMotorControl, motor] = ...
    SetupMotorControlSerial(motorControlCOM);
clc;

% Sets up the kinematics
controlPoint = 6;
KC = Robot.KinematicChains.MAK;
KC.optimization.weightings(controlPoint) = 100;
pointsd = zeros(4, size(KC.points.kP,2));

% Trajectory parameters
traj.traj = 3;
traj.delay = 0.4;
traj.motor = motor;
traj.serialMotorControl = serialMotorControl;
traj = TrajectoriesMechatronicArm(0, traj);

% Status report options
status.count = 0;
status.point = 1:6;

% Open gripper to start
serialMotorControl.servoWrite(motor.gripper, 0);
pause(0.5);

while (states.run)
    % Runs the trajectory multiple times
    for i = 1:traj.velocity:traj.runs
        
        % Get current states
        states = guidata(RobotFigure);
        if (~states.run), break; end;
        
        % Create the trajectory
        traj = TrajectoriesMechatronicArm(i, traj);
        
        % Adds noise to trajectory and carries out inverse
        % kinematics on the noisy trajectory
        pointsd(:,controlPoint) = [traj.point; 1];
        X = InverseKinematicOptimization(KC,pointsd);
        
        % Move arm servos
        MechatronicArmControl(serialMotorControl, motor, X);
        
        % Rotates and plots the right arm to optimized value
        KC = RotateKinematicChain(KC,X);
        Robot.KinematicChains.MAK = KC;
        RobotPlot(Robot);
        drawnow;
        
        % Prints the angle and point status report of the KC
        status = PrintStatusReport(KC, X, status);
    end
end