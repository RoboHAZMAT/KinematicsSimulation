function [qR, resetR, qL, resetL] = CheckIMUReading(serialObjIMU, qR, resetR, qL, resetL)
%% ==========================Check IMU Reading=============================
% RoboHAZMAT: Senior Design Project
% Motion Control Team
% Gerardo Bledt
% January 2, 2015
%
% Makes sure that the IMU readings are valid from all sensors.

% Check readings from the right arm IMUs
while (isnan(qR(1,1)) || isnan(qR(2,1)))
    [qR(1,:), resetR(1)] = ReadIMUQuaternion(serialObjIMU(1));
    [qR(2,:), resetR(2)] = ReadIMUQuaternion(serialObjIMU(2));
end

% Check readings from the left arm IMUs
% while (isnan(qL(1,1)) || isnan(qL(2,1)))
%     [qL(1,:), resetR(1)] = ReadIMUQuaternion(serialObjIMU(3));
%     [qL(2,:), resetR(2)] = ReadIMUQuaternion(serialObjIMU(4));
% end