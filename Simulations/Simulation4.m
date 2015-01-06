function Robot = Simulation4(Robot)
%% =================IMU Controlled Robot Arm Simulation====================

% Sets up the Serial communication with the IMUs
IMUCOM = SetupCOM;
serialObjIMU(1) = SetupIMUSerial(IMUCOM(1,:));
serialObjIMU(2) = SetupIMUSerial(IMUCOM(2,:));
% serialObjIMU(3) = SetupIMUSerial(IMUCOM(3,:));
% serialObjIMU(4) = SetupIMUSerial(IMUCOM(4,:));

% Specifies the arm and points to be controlled
% *** This is what must be tuned ***
% Right Arm Control Gains
KCR = Robot.KinematicChains.RMK;
KCR.optimization.weightings(4) = 300;
KCR.optimization.weightings(5) = 100;
KCR.optimization.weightings(6) = 200;
KCR.optimization.weightings(7) = 500;

% Left Arm Control Gains
KCL = Robot.KinematicChains.LMK;
KCL.optimization.weightings(4) = 300;
KCL.optimization.weightings(5) = 100;
KCL.optimization.weightings(6) = 200;
KCL.optimization.weightings(7) = 500;

% Sets up the estimated actual arm position
shoulderR = [0,-0.179,0.371];
shoulderL = [0,0.179,0.371];
link(1,:) = [.279,0,0];
link(2,:) = [0,0,-0.076];
link(3,:) = [0.257,0,0];
link(4,:) = [0,0,-0.076];

% Yaw angle offset initialization at zero
psiR = zeros(1,3);
psiL = zeros(1,3);

% Wait for user to be ready
begin = '';
while (~strcmpi(begin,'y') && ~strcmpi(begin,'n'))
    begin = input('Begin control? [Y/N]: ','s');
    if (strcmpi(begin,'y'))
        disp(' '); disp(' ');
        disp('   Robot ARM Control   ');
        disp(' ');
    end
end

% Gets the two IMU readings from the sensors
qR(1,:) = ReadIMUQuaternion(serialObjIMU(1));
qR(2,:) = ReadIMUQuaternion(serialObjIMU(2));
%     [qL(1,:), resetL(1)] = ReadIMUQuaternion(serialObjIMU(3));
%     [qL(2,:), resetL(2)] = ReadIMUQuaternion(serialObjIMU(4));

% Forces a reset
resetR = [1, 1];
% resetL = [1, 1];

% 2. Estimates the orientation of the arm links
[linkRRot, psiR] = ...
    EstimateArmOrientation(link, qR, resetR, psiR);
%     [linkLRot, psiL] = ...
%         EstimateArmOrientation(link, qL, resetL, psiL);

% Constant running while loop
% 1. Read IMU for quaternions
% 2. Estimate arm link orientations
% 3. Reconstruct arm positioning
% 4. Inverse Kinematic Optimization to estimate joint angles
% 5. Rotate arm kinematic chain
while (strcmpi(begin,'y'))
    % 1. Gets the two IMU readings from the sensors
    [qR(1,:), resetR(1)] = ReadIMUQuaternion(serialObjIMU(1));
    [qR(2,:), resetR(2)] = ReadIMUQuaternion(serialObjIMU(2));
%     [qL(1,:), resetL(1)] = ReadIMUQuaternion(serialObjIMU(3));
%     [qL(2,:), resetL(2)] = ReadIMUQuaternion(serialObjIMU(4));
    
    % 2. Estimates the orientation of the arm links
    [linkRRot, psiR] = ...
        EstimateArmOrientation(link, qR, resetR, psiR);
%     [linkLRot, psiL] = ...
%         EstimateArmOrientation(link, qL, resetL, psiL);
    
    % 3. Reconstructs the user's arm and desired points
    pointsdR = ReconstructArm(shoulderR, linkRRot);
%     pointsdL = ReconstructArm(shoulderL, linkLRot);
    
    % 4. Inverse Kinematic optimization for joint angles
    XR = InverseKinematicOptimization(Robot, 'RMK', pointsdR);
%     XL = InverseKinematicOptimization(Robot,'LMK',pointsdL);
    
    % 5. Rotates and plots the right arm to optimized value
    Robot.KinematicChains.RMK = RotateKinematicChain(KCR, XR);
%     Robot.KinematicChains.LMK = RotateKinematicChain(KCL, XL);
    RobotPlot(Robot);
    drawnow;
end