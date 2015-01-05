function Robot = Simulation9(Robot)

% Initializes communication with Mechatronic Arm
[IMUCOM, motorControlCOM] = SetupCOM;
[serialMotorControl, motor] = ...
    SetupMotorControlSerial(motorControlCOM);

% Sets up the kinematics
KC = Robot.KinematicChains.MAK;
pointsd = zeros(4, size(KC.points.p,2));

% History vectors for trajectories
xHist = zeros(1,360);
yHist = zeros(1,360);
zHist = zeros(1,360);
xHistT = zeros(1,360);
yHistT = zeros(1,360);
zHistT = zeros(1,360);

% Set velocity: {1,..,.5}
velocity = 5;
n = 60;

while (1)
    serialMotorControl.servoWrite(motor.gripper, 0);
    pause(0.5);
    % Runs the trajectory multiple times
    for i = 1:velocity:(13*n - 1)
        
        % Create the trajectory
        [x, y, z] = TrajectoriesMechatronicArm(i, 3, serialMotorControl, motor);
        
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
        
        % plot3(xHist,yHist,zHist,'.','color','green');
        % plot3(xHistT,yHistT,zHistT,'.','color','red');
        drawnow;
        
        MechatronicArmControl(serialMotorControl, motor, X);
    end
    
end

