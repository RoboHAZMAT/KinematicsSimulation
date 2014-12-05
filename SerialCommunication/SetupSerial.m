function serialObjIMU = SetupSerial

% Specify COM port for the IMU
IMUCOM = 'COM8';

% Close and delete all COM ports
if (instrfindall ~= [])
    fclose(instrfindall);
end
delete(instrfindall);

% Setup Serial Communication with IMU
disp('Setup Serial Communication...');
serialObjIMU = serial(IMUCOM,'BAUD',38400,'InputBufferSize',256);%,'Timeout',0.25,'InputBufferSize',512);
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
    readingIMU = fscanf(serialObjIMU);
    %disp(readingIMU);
end
% Change input size to match incoming IMU data
%while (1)
[yaw, pitch, roll, readingIMU] = ReadIMU(serialObjIMU);
%disp(readingIMU);
%disp(yaw);
%disp(pitch);
%end