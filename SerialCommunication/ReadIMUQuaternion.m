function [qw, qx, qy, qz, reset, readingIMU] = ReadIMUQuaternion(serialObjIMU)
qw = NaN;
qx = NaN;
qy = NaN;
qz = NaN;
reset = NaN;
readingIMU = NaN;

while(serialObjIMU.BytesAvailable > 0)
    readingIMU = fscanf(serialObjIMU);
end

if (~isnan(readingIMU))
    
    pos1 = strfind(readingIMU, '$');
    pos2 = strfind(readingIMU, '#');
    pos3 = strfind(readingIMU, '%');
    pos4 = strfind(readingIMU, '&');
    pos5 = strfind(readingIMU, '@');
    pos6 = strfind(readingIMU, '!');
    
    qw = str2double(readingIMU(pos1 + 1:pos2 - 1));
    qx = str2double(readingIMU(pos2 + 1:pos3 - 1));
    qy = str2double(readingIMU(pos3 + 1:pos4 - 1));
    qz = str2double(readingIMU(pos4 + 1:pos5 - 1));
    reset = str2double(readingIMU(pos5 + 1:pos6 - 1));
end