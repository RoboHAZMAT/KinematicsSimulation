function serialObjIMU = SetupIMUSerial(IMUCOM)
%% =======================Serial IMU Communication=========================
% RoboHAZMAT: Senior Design Project
% Motion Control Team
% Gerardo Bledt
% December 15, 2014
%
% Ability to connect to the IMU sensor and read in data over Serial.

% Setup Serial Communication with IMU
disp('Setup Serial Communication...');
serialObjIMU = serial(IMUCOM,'BAUD',38400,'InputBufferSize',256);
fopen(serialObjIMU);
pause(2);

disp('Initializing IMU...');
% Clear received message in buffer
while (strcmp(serialObjIMU.fscanf,'Initializing I2C devices...'))
end
serialObjIMU.fscanf;

% Test Connections
disp('IMU Connection Successful. Testing Device Connections...');
while (strcmp(serialObjIMU.fscanf,'Testing device connections...'))
end
serialObjIMU.fscanf;

% Initialize DMP
disp('Device Connection Successful. Initializing DMP...');
while (strcmp(serialObjIMU.fscanf,'Initializing DMP...'))
end
serialObjIMU.fscanf;

% Enable DMP
disp('DMP Connection Successful. Enabling DMP...');
while (strcmp(serialObjIMU.fscanf,'Enabling DMP...'))
end
disp('DMP Enabled. Ready for use.');

% Clear current buffer
while (serialObjIMU.BytesAvailable > 0)
    fscanf(serialObjIMU);
end
