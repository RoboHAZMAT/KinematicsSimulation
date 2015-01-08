function Robot = SimulationM2(Robot)
%% ==========IMU controlled Mechatronic Arm Through Arduino (ypr)==========

% Sets up the Keyboard Control
[RobotFigure, states] = SetupKeyboardControl;

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

% Constant running while loop
while (states.run)
    % Get current states
    states = guidata(RobotFigure);
    
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
        
        % Rotates and plots the Robot Kinematics
        KC = RotateKinematicChain(KC,X);
        Robot.KinematicChains.MAK = KC;
        RobotPlot(Robot);
        drawnow;
    end
end