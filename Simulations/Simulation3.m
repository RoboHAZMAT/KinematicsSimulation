function Robot = Simulation3(Robot)

% Sets up the COM ports
IMUCOM = SetupCOM;

% Left Manipulator simulation controlled by IMU
serialObjIMU = SetupIMUSerial(IMUCOM(1,:));

KC = Robot.KinematicChains.LMK;
%calibrateIMU();
while (mode == 3)
    % Reads the IMU
    [yaw, pitch, roll, readingIMU] = ReadIMU(serialObjIMU);
    if (~isnan(readingIMU))
        % Rotation vector
        X = zeros(6,1);
        X(2,1) = (-(yaw))/180*pi;
        X(4,1) = (-(pitch + 90))/180*pi;
        X(5,1) = (-(roll))/180*pi;
        X(6,1) = (90)/180*pi;
        
        % Rotates and plots the kinematic chain
        KC = RotateKinematicChain(KC,X);
        Robot.KinematicChains.LMK = KC;
        RobotPlot(Robot);
        drawnow;
    end
end