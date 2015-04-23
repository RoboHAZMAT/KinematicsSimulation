function [IMUCOM, motorControlCOM, headControlCOM, wirelessIMUCOM, arbotixCOM] = SetupCOM
%% ============================Setup COM Ports=============================
% RoboHAZMAT: Senior Design Project
% Motion Control Team
% Gerardo Bledt
% December 15, 2014
%
% Setup for the COM ports in the computer communicating with the IMU and
% the Arduino for motor control. Add more COM# strings to add ports.

% Close and delete all COM ports
if (~isempty(instrfindall))
    fclose(instrfindall);
end
delete(instrfindall);

% Specify COM port for the IMU
% Right Lower Arm, Right Upper Arm
IMUCOM{1} = 'COM9';
IMUCOM{2} = 'COM8';

% Specify COM port for the Motor Control
motorControlCOM = 'COM11';

% Specify COM port for the Head Control
headControlCOM = 'COM20';

% Wireless IMU COM port
wirelessIMUCOM = 'COM8';

% Arbotix COM port
arbotixCOM = 'COM10';