function calibrated = CalibrateIMU(serialObjIMU,nIMU)
%% ============================Calibrate IMU===============================
% RoboHAZMAT: Senior Design Project
% Motion Control Team
% Gerardo Bledt
% January 15, 2014
%
% A rough calibration method that detects when the IMU readings stabilize.

% Initializes the quaternion buffer
qBuffer = 1:150;

% Attempts to callibrate 10 times
for j = 1:10
    
    % Fills the buffer with 150 readings
    fprintf('..');
    for i = 1:150
        q = ReadWirelessIMU(serialObjIMU,nIMU);
        qBuffer(i) = 100*q(4);
    end
    
    % If the reading has stabilized, calibration is complete
    if (max(qBuffer) - min(qBuffer) >= 1)
        calibrated = false;
    else
        calibrated = true;
        break;
    end
end