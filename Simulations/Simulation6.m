function Robot = Simulation6(Robot)

% Sets up the Serial communication with the IMUs
IMUCOM = SetupCOM;
serialObjIMU(1) = SetupIMUSerial(IMUCOM(1,:));
serialObjIMU(2) = SetupIMUSerial(IMUCOM(2,:));
% serialObjIMU(3) = SetupIMUSerial(IMUCOM(3,:));
% serialObjIMU(4) = SetupIMUSerial(IMUCOM(4,:));

% Specifies the arm and points to be controlled
KCR = Robot.KinematicChains.RMK;
KCR.weightings(4) = 300;
KCR.weightings(5) = 100;
KCR.weightings(6) = 200;
% KCL = Robot.KinematicChains.LMK;
% KCL.weightings(4) = 300;
% KCL.weightings(5) = 100;
% KCL.weightings(6) = 200;

% Sets up the estimated actual arm position
shoulderR = [0,-0.179,0.371];
% shoulderL = [0,0.179,0.371];
link(1,:) = [.279,0,0];
link(2,:) = [0.257,0,0];
link(3,:) = [0,0,-0.076];

% Yaw angle offset initialization at zero
psiR = zeros(1,3);
% psiL = zeros(1,3);

% Wait for user to be ready
begin = 'n';
while (~strcmpi(begin,'y'))
    begin = input('\nBegin control? [Y/N]: ','s');
end

% Constant running while loop
% 1. Read IMU for quaternions
% 2. Estimate arm link orientations
% 3. Reconstruct arm positioning
% 4. Inverse Kinematic Optimization to estimate joint angles
% 5. Rotate arm kinematic chain
while (1)
    tic
    % 1. Gets the two IMU readings from the sensors
    [qR(1,:), resetR(1)] = ReadIMUQuaternion(serialObjIMU(1));
    [qR(2,:), resetR(2)] = ReadIMUQuaternion(serialObjIMU(2));
    % [qL(1,:), resetL(1)] = ReadIMUQuaternion(serialObjIMU(3));
    % [qL(2,:), resetL(2)] = ReadIMUQuaternion(serialObjIMU(4));
    
    % 2. Estimates the orientation of the arm links
    [linkRRot, psiR] = ...
        EstimateArmOrientation(link, qR, resetR, psiR);
    % [linkLRot, psiL] = ...
    %     EstimateArmOrientation(link, qL, resetL, psiL);
    
    % 3. Reconstructs the user's arm and desired points
    pointsdR = ReconstructArm(shoulderR, linkRRot);
    % pointsdL = ReconstructArm(shoulderL, linkLRot);
    
    % 4. Inverse Kinematic optimization for joint angles
    XR = InverseKinematicOptimization(Robot, 'RMK', pointsdR);
    % XL = InverseKinematicOptimization(Robot,'LMK',pointsdL);
    
    % 5. Rotates and plots the right arm to optimized value
    Robot.KinematicChains.RMK = RotateKinematicChain(KCR, XR);
    % Robot.KinematicChains.LMK = RotateKinematicChain(KCL, XL);
    RobotPlot(Robot);
    drawnow;
    toc
end