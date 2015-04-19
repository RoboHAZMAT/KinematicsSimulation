function [serialHeadControl, motor] = ...
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
serialHeadControl = arduino(headControlCOM);

% Define motor PWM pins
motor = struct();
motor.neckYaw = 6;
motor.neckPitch = 5;
motor.neckRoll = 3;

% Attach servo motors to their pins
serialHeadControl.servoAttach(motor.neckYaw);
serialHeadControl.servoAttach(motor.neckPitch);
serialHeadControl.servoAttach(motor.neckRoll);

RobotHeadControl(serialHeadControl, motor, zeros(3,1));