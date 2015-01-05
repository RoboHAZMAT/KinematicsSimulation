function Robot = Simulation7(Robot)

% Setup COM ports
[IMUCOM, motorControlCOM] = SetupCOM;

% Setup communication with IMU and Arm
serialObjIMU = SetupIMUSerial(IMUCOM(1,:));
[serialMotorControl, motor] = ...
    SetupMotorControlSerial(motorControlCOM);

% Constrain unused motors to the given values
serialMotorControl.servoWrite(motor.elbowPitch, round(150));
serialMotorControl.servoWrite(motor.wristPitch, round(30));
%serialMotorControl.servoWrite(motor.wristRoll, round(110));

% Get the kinematic chain
KC = Robot.KinematicChains.MAK;
X = zeros(5,1);

% Constant running while loop:
% To quit: 'Ctrl+C' (May try to find a better way)
while (1)
    
    % Read the IMU sensor
    [yaw, pitch, roll, readingIMU] = ReadIMU(serialObjIMU);
    
    % If a reading is obtained, rotate simulation and arm
    if (~isnan(readingIMU))
        % Rotates the base yaw, base pitch, and wrist roll
        serialMotorControl.servoWrite(motor.baseYaw, ...
            round(yaw));
        serialMotorControl.servoWrite(motor.basePitch, ...
            round(pitch));
        serialMotorControl.servoWrite(motor.wristRoll, ...
            round(180-roll));
        
        % Rotate and plot the simulated Mechatronic Arm
        X(1,1) = (-(yaw))/180*pi;
        X(2,1) = (-pitch + 90)/180*pi;
        X(3,1) = (60)/180*pi;
        X(4,1) = (30)/180*pi;
        X(5,1) = (-(roll))/180*pi;
        KC = RotateKinematicChain(KC,X);
        Robot.KinematicChains.MAK = KC;
        RobotPlot(Robot);
        drawnow;
    end
end