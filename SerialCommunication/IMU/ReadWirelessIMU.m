function [q, reset, readingIMU] = ReadWirelessIMU(serialObjIMU, nIMU)
%% =========================Read IMU Quaternion============================
% RoboHAZMAT: Senior Design Project
% Motion Control Team
% Gerardo Bledt
% December 30, 2014
%
% Reads in the incoming data from the IMU sensor which is processed by an
% Arduino with the IMUQuat.ino program uploaded on it. Uses the
% communication protocol to identify the four describing parameters of the
% quaternion in a reading is available.

% Initializes the readings
qw = NaN;
qx = NaN;
qy = NaN;
qz = NaN;
reset = 0;
readingIMU = NaN;

flushinput(serialObjIMU);
while (isnan(qw) || isnan(qx) || isnan(qy) || isnan(qz))
    fwrite(serialObjIMU, nIMU); % possible change for fprintf, but fprintf is slower I think
    
    % Reads the incoming buffer stream and clears it
    if (serialObjIMU.BytesAvailable > 0)
        readingIMU = fscanf(serialObjIMU);
    end
    
    % If a reading is found from the IMU
    if (~isnan(readingIMU))
        
        % Finds the positions of the identifying markers
        %pos1 = strfind(readingIMU, '*');
        %pos2 = strfind(readingIMU, '^');
        pos3 = strfind(readingIMU, '$');
        pos4 = strfind(readingIMU, '#');
        pos5 = strfind(readingIMU, '%');
        pos6 = strfind(readingIMU, '&');
        pos7 = strfind(readingIMU, '@');
        pos8 = strfind(readingIMU, '!');
        
        % Pulls the four quaternion parameters out of the reading
        qw = str2double(readingIMU(pos3 + 1:pos4 - 1));
        qx = str2double(readingIMU(pos4 + 1:pos5 - 1));
        qy = str2double(readingIMU(pos5 + 1:pos6 - 1));
        qz = str2double(readingIMU(pos6 + 1:pos7 - 1));
        reset = str2double(readingIMU(pos7 + 1:pos8 - 1));
        q = [qw, qx, qy, qz];
        if (isnan(reset)), reset = 0; end;
    end
end