function serialObjIMU = SetupIMUSerial(IMUCOM)
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
fprintf('Setup Serial Communication With %s...\n', IMUCOM);
%SetSimulationControlText(states,'','Setup Serial Communication...','','');

% Check if COM port is available
if (~ismember(GetAvailableCOM,IMUCOM))
    %SetSimulationControlText(states,'','COM Port not available.','','');
    error('%s is not available',IMUCOM);
end

% Create and open Serial communication
serialObjIMU = serial(IMUCOM,'BAUD',baudrate,'InputBufferSize',90);
fopen(serialObjIMU);
initFailed = 0;
pause(0.5);

% fprintf('Initializing IMU...\n');
% %SetSimulationControlText(states,'','Setup Serial Communication...',...
% %    'Initializing IMU...','');
% % Clear received message in buffer
% while (~initFailed && ...
%         isempty(strfind(serialObjIMU.fscanf,'Initializing I2C devices...')))
%     fprintf('Initialization Failed. Retry...\n\n'); initFailed = 1;
%     %SetSimulationControlText(states,'Setup Serial Communication...','',...
% %        'Initialization Failed.','Retry...'); break;
% end

% % Test Connections
% if (~initFailed)
%     fprintf('IMU Connection Successful. Initializing DMP...\n'); end;
% %SetSimulationControlText(states,'Setup Serial Communication...','',...
% %    'IMU Connection Successful.','Initializing DMP...');
% while (~initFailed && ...
%         isempty(strfind(serialObjIMU.fscanf,'Initializing DMP...')))
%     fprintf('Initialization Failed. Retry...\n\n'); initFailed = 1; break;
% end
% 
% % Enable DMP
% if (~initFailed)
%     fprintf('DMP Connection Successful. Enabling DMP...\n'); end;
% while (~initFailed && ...
%         isempty(strfind(serialObjIMU.fscanf,'Enabling DMP...')))
%     fprintf('Initialization Failed. Retry...\n\n'); initFailed = 1; break;
% end

% Clear current buffer

while (~initFailed && isempty(strfind(serialObjIMU.fscanf,'System is ready')))
    fprintf('Initialization Failed. Retry...\n\n'); initFailed = 1; break;
end
    
% while (serialObjIMU.BytesAvailable > 0)
%     fscanf(serialObjIMU);
% end
if (~initFailed)
    fprintf('System is Ready. Calibrating IMU...');
    calibrated = CalibrateIMU(serialObjIMU,'2');
    if (calibrated)
        fprintf('.\nReady to use!\n\n');
    else
        fprintf('Calibration Failed. Retry...\n\n');
        delete(serialObjIMU); serialObjIMU = SetupIMUSerial(IMUCOM);
    end
else
    delete(serialObjIMU); serialObjIMU = SetupIMUSerial(IMUCOM);
end