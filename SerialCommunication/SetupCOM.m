function [IMUCOM, motorControlCOM] = SetupCOM
% Setup for the COM ports in the computer communicating with the IMU and
% the Arduino for motor control.

% Specify COM port for the IMU
IMUCOM = 'COM9';

% Specify COM port for the Motor Control
motorControlCOM = 'COM11';