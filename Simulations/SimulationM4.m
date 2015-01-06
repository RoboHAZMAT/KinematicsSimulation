function Robot = SimulationM4(Robot)

% Sets up the Serial communication with the IMUs and motor control
[IMUCOM, motorControlCOM] = SetupCOM;
serialObjIMU(1) = SetupIMUSerial(IMUCOM(1,:));
serialObjIMU(2) = SetupIMUSerial(IMUCOM(2,:));
[serialMotorControl, motor] = ...
    SetupMotorControlSerial(motorControlCOM);

% Get the kinematic chain
KC = Robot.KinematicChains.MAK;
KC.weightings(4) = 300;
KC.weightings(5) = 300;

% Sets up the estimated actual arm position
shoulder = [0,-0.179,0.371];
link(1,:) = [.279,0,0];
link(2,:) = [0.257,0,0];
link(3,:) = [0,0,-0.076];

% Yaw angle offset initialization at zero
psi = zeros(1,3);

% Wait for user to be ready
begin = 'n';
while (~strcmpi(begin,'y'))
    begin = input('\nBegin control? [Y/N]: ','s');
end

% Constant running while loop
% 1. Read IMU for quaternions
% 2. Estimate arm link orientations
% 3. Reconstruct arm positioning
% 4. Translate points from human to mechatronic kinematics
% 5. Inverse Kinematic Optimization to estimate joint angles
% 6. Rotate arm kinematic chain
% 7. Move mechatronic arm motors
while (1)
    % 1. Gets the two IMU readings from the sensors
    [q(1,:), reset(1)] = ReadIMUQuaternion(serialObjIMU(1));
    [q(2,:), reset(2)] = ReadIMUQuaternion(serialObjIMU(2));
    
    % 2. Estimates the orientation of the arm links
    [linkRot, psi] = ...
        EstimateArmOrientation(link, q, reset, psi);
    
    % 3. Reconstructs the user's arm and desired points
    pointsd = ReconstructArm(shoulder, linkRot);
    
    % 4. Translate points between human arm and mechatronic arm
    pointsd = MechatronicArmTranslate(KC, pointsd);
    
    % 5. Inverse Kinematic optimization for joint angles
    X = InverseKinematicOptimization(Robot, 'MAK', pointsd);
    
    % 6. Rotates and plots the right arm to optimized value
    Robot.KinematicChains.MAK = RotateKinematicChain(KC, X);
    RobotPlot(Robot);
    drawnow;    
    
    % 7. Move Mechatronic Arm motors
    MechatronicArmControl(serialMotorControl, motor, X);
end