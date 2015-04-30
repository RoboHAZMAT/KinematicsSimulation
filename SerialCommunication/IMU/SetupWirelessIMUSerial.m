function serialObjWirelessIMU = SetupWirelessIMUSerial(wirelessIMUCOM, nIMU, receiveIMU)
%% =======================Serial IMU Communication=========================
% RoboHAZMAT: Senior Design Project
% Motion Control Team
% Gerardo Bledt
% December 15, 2014
%
baudrate = 115200; %Added for easy change

% Ability to connect to the IMU sensor and read in data over Serial.
states = guidata(gcf);

% Setup Serial Communication with IMU
fprintf('Setup Serial Communication With %s...', wirelessIMUCOM);
%SetSimulationControlText(states,'','Setup Serial Communication...','','');

% Check if COM port is available
if (~ismember(GetAvailableCOM,wirelessIMUCOM))
    %SetSimulationControlText(states,'','COM Port not available.','','');
    error('%s is not available',wirelessIMUCOM);
end

% Create and open Serial communication
serialObjWirelessIMU = serial(wirelessIMUCOM,'BAUD',baudrate,'InputBufferSize',90);
fopen(serialObjWirelessIMU);
initFailed = 0;
pause(1);fprintf('...');pause(1);fprintf('...');pause(1);fprintf('...');
fwrite(serialObjWirelessIMU, receiveIMU);
pause(1);fprintf('...\n');

fprintf('Turn on the IMU!\n');
stri = serialObjWirelessIMU.fscanf;
%stri = 'System is ready';
while (~initFailed && isempty(strfind(stri,'System is ready')))
    stri = serialObjWirelessIMU.fscanf;
    fprintf('Initialization Failed. Retry...\n\n'); initFailed = 1; break;
end

% while (serialObjIMU.BytesAvailable > 0)
%     fscanf(serialObjIMU);
% end
if (~initFailed)
    fprintf('System is Ready.');
    for i = 1:length(nIMU)
        fprintf('\nCalibrating IMU...');
        calibrated = CalibrateWirelessIMU(serialObjWirelessIMU, nIMU(i));
        if (calibrated)
            fprintf('.\nReady to use!\n');
        else
            fprintf('Calibration Failed. Retry...\n\n');
            break;
        end
    end
    if (~calibrated)
        delete(serialObjWirelessIMU); 
        serialObjWirelessIMU = SetupWirelessIMUSerial(wirelessIMUCOM, nIMU, receiveIMU);
    end
else
    delete(serialObjWirelessIMU); 
    serialObjWirelessIMU = SetupWirelessIMUSerial(wirelessIMUCOM, nIMU, receiveIMU);
end