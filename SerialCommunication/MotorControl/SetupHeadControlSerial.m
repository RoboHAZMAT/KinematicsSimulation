function [serialMotorControl, motor] = ...
    SetupHeadControlSerial(headControlCOM)
%% ======================Setup Motor Control Serial========================
% RoboHAZMAT: Senior Design Project
% Motion Control Team
% Gerardo Bledt
% December 29, 2014
%
% Sets up the communication with the Arduino motor controller and joint
% servos for the Mechatronic Arm over Serial.

% Check if COM port is available
if (~ismember(GetAvailableCOM,headControlCOM))
    error('%s is not available',headControlCOM);
end

% Open communication with the Arduino
serialMotorControl = arduino(headControlCOM);

% Define motor PWM pins
motor = struct();
motor.neckYaw = 3;
motor.neckPitch = 5;
motor.neckRoll = 6;

% Attach servo motors to their pins
serialMotorControl.servoAttach(motor.neckYaw);
serialMotorControl.servoAttach(motor.neckPitch);
serialMotorControl.servoAttach(motor.neckRoll);

RobotHeadControl(serialMotorControl, motor, zeros(3,1));