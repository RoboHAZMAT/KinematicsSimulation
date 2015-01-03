function Robot = Simulation2(Robot)

% Sets up COM ports
IMUCOM = SetupCOM;

% Right Manipulator simulation controlled by IMU
serialObjIMU = SetupIMUSerial(IMUCOM(1,:));

KC = Robot.KinematicChains.RMK;
while (1)
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
        Robot.KinematicChains.RMK = KC;
        RobotPlot(Robot);
        drawnow;
    end
end