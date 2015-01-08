function Robot = Simulation3(Robot)
%% ================IMU Controlled Manipulators Simulation==================

% Sets up the Keyboard Control
[RobotFigure, states] = SetupKeyboardControl;

% Sets up the COM ports
IMUCOM = SetupCOM;

% Setup IMU Serial communication
serialObjIMU(1) = SetupIMUSerial(IMUCOM(1,:));
serialObjIMU(2) = SetupIMUSerial(IMUCOM(2,:));

% Initializes KCs
KCR = Robot.KinematicChains.RMK;
KCL = Robot.KinematicChains.LMK;

% Setup offests
psiR = 0;
psiL = 0;

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

%calibrateIMU();
while (strcmpi(begin,'y') && states.run)
    % Reads the IMUs
    [qR, resetR] = ReadIMUQuaternion(serialObjIMU(1));
    [qL, resetL] = ReadIMUQuaternion(serialObjIMU(2));
    
    % Convert quaternions to yaw, pitch, roll
    yprR = QuaternionToYPR(qR);
    yprL = QuaternionToYPR(qL);
    
    % Reset the yaws if requested 
    if (resetR)
        psiR = yprR(1);
    end
    if (resetL)
        psiL = yprL(1);        
    end
    yprR(1) = yprR(1) - psiR;
    yprL(1) = yprL(1) - psiL;
    
    % Rotation vector for RMK
    XR = zeros(6,1);
    XR(2,1) = (-(yprR(1)));
    XR(4,1) = (-(yprR(2) + pi/2));
    XR(5,1) = (-(yprR(3)));
    XR(6,1) = pi/2;
    
    % Rotation vector for RMK
    XL = zeros(6,1);
    XL(2,1) = (-(yprL(1)));
    XL(4,1) = (-(yprL(2) + pi/2));
    XL(5,1) = (-(yprL(3)));
    XL(6,1) = pi/2;
    
    % Rotates and plots the kinematic chain
    KCR = RotateKinematicChain(KCR,XR);
    Robot.KinematicChains.RMK = KCR;
    KCL = RotateKinematicChain(KCL,XL);
    Robot.KinematicChains.LMK = KCL;
    RobotPlot(Robot);
    drawnow;
    
    % Gets simulation state
    states = guidata(RobotFigure);
end