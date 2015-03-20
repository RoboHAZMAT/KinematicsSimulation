function [IMUCOM, motorControlCOM, headControlCOM] = SetupCOM
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
IMUCOM{1} = 'COM16';
IMUCOM{2} = 'COM20';

% Specify COM port for the Motor Control
motorControlCOM = 'COM11';

% Specify COM port for the Head Control
headControlCOM = 'COM19';