function [yaw, pitch, roll, readingIMU] = ReadIMU(serialObjIMU)
error = true;
yaw = NaN;
pitch = NaN;
roll = NaN;
readingIMU = NaN;

% Clear received message in buffer
while(serialObjIMU.BytesAvailable <= 0)
end
while(serialObjIMU.BytesAvailable > 0)
    readingIMU = fscanf(serialObjIMU);
end

if (~isnan(readingIMU))
    pos1 = strfind(readingIMU, '$');
    pos2 = strfind(readingIMU, '#');
    pos3 = strfind(readingIMU, '%');
    pos4 = strfind(readingIMU, '&');
    
    yaw = str2double(readingIMU(pos1 + 1:pos2 - 1));
    pitch = str2double(readingIMU(pos2 + 1:pos3 - 1));
    roll = str2double(readingIMU(pos3 + 1:pos4 - 1));
end