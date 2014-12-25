function [IMUCOM, motorControlCOM] = SetupCOM
% Setup for the COM ports in the computer communicating with the IMU and
% the Arduino for motor control.

% Close and delete all COM ports
if (~isempty(instrfindall))
    fclose(instrfindall);
end
delete(instrfindall);

% Specify COM port for the IMU
IMUCOM = ['COM8';'COM9'];

% Specify COM port for the Motor Control
motorControlCOM = 'COM11';
