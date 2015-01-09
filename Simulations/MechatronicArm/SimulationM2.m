function Robot = SimulationM2(Robot)
%% ==========IMU controlled Mechatronic Arm Through Arduino (ypr)==========

% Sets up the Keyboard Control
[RobotFigure, states] = SetupKeyboardControl;

% Setup COM ports
[IMUCOM, motorControlCOM] = SetupCOM;

% Setup communication with IMU and Arm
serialObjIMU = SetupIMUSerial(IMUCOM(2,:));
[serialMotorControl, motor] = ...
    SetupMotorControlSerial(motorControlCOM);

% Constrain unused motors to the given values
serialMotorControl.servoWrite(motor.elbowPitch, round(150));
serialMotorControl.servoWrite(motor.wristPitch, round(30));
%serialMotorControl.servoWrite(motor.wristRoll, round(110));

% Get the kinematic chain
KC = Robot.KinematicChains.MAK;
X = zeros(5,1);
psi = 0;

% Constant running while loop
while (states.run)
    % Get current states
    states = guidata(RobotFigure);
    
    % Read the IMU sensor
    [q, reset, readingIMU] = ReadIMUQuaternion(serialObjIMU);
    ypr = QuaternionToYPR(q);
    
    % If a reading is obtained, rotate simulation and arm
    if (~isnan(readingIMU))
%         % Rotates the base yaw, base pitch, and wrist roll
%         serialMotorControl.servoWrite(motor.baseYaw, ...
%             round(ypr(1)));
%         serialMotorControl.servoWrite(motor.basePitch, ...
%             round(ypr(2)));
%         serialMotorControl.servoWrite(motor.wristRoll, ...
%             round(180-ypr(3)));
        
        if (reset), psi = ypr(1); end;
        
        % Rotate and plot the simulated Mechatronic Arm
        X(1,1) = (-(ypr(1) - psi));
        X(2,1) = (-ypr(2));
        X(3,1) = (65)*pi/180;
        X(4,1) = (65)*pi/180;
        X(5,1) = (-(ypr(3)));
        
        MechatronicArmControl(serialMotorControl, motor, X)
        
        % Rotates and plots the Robot Kinematics
        KC = RotateKinematicChain(KC,X);
        Robot.KinematicChains.MAK = KC;
        RobotPlot(Robot);
        drawnow;
    end
end