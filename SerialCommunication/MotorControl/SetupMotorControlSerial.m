function [serialMotorControl, motor] = ...
    SetupMotorControlSerial(motorControlCOM)
%% ======================Setup Motor Control Serial========================
% RoboHAZMAT: Senior Design Project
% Motion Control Team
% Gerardo Bledt
% December 29, 2014
%
% Sets up the communication with the Arduino motor controller and joint
% servos for the Mechatronic Arm over Serial.

% Check if COM port is available
if (~ismember(GetAvailableCOM,motorControlCOM))
    error('%s is not available',motorControlCOM);
end

% Open communication with the Arduino
serialMotorControl = arduino(motorControlCOM);

% Define motor PWM pins
motor = struct();
motor.wristRoll = 3;
motor.wristPitch = 5;
motor.elbowPitch = 6;
motor.basePitch = 9;
motor.baseYaw = 10;
motor.gripper = 11;

% Attach servo motors to their pins
serialMotorControl.servoAttach(motor.baseYaw);
serialMotorControl.servoAttach(motor.basePitch);
serialMotorControl.servoAttach(motor.elbowPitch);
serialMotorControl.servoAttach(motor.wristPitch);
serialMotorControl.servoAttach(motor.wristRoll);
serialMotorControl.servoAttach(motor.gripper);

MechatronicArmControl(serialMotorControl, motor, zeros(5,1));