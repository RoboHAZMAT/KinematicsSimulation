function [ypr] = QuaternionToYPR(q)
%% ===================Quaternion to Yaw, Pitch, Roll=======================
% RoboHAZMAT: Senior Design Project
% Motion Control Team
% Gerardo Bledt
% January 8, 2015
%
% Converts the quaternion into yaw, pitch, and roll angles. Subject to the
% Gimbal lock issue. Using quaternions to carry out rotations is more
% advisable, but this allows for quick simple rough rotations.

% Bound the quaternion
if (q(1) >= 2), q(1) = -4 + q(1); end;
if (q(2) >= 2), q(2) = -4 + q(2); end;
if (q(3) >= 2), q(3) = -4 + q(3); end;
if (q(4) >= 2), q(4) = -4 + q(4); end;

% Calculates the gravity vector from the quaternion
grav(1) = 2 * (q(2)*q(4) - q(1)*q(3));
grav(2) = 2 * (q(1)*q(2) + q(3)*q(4));
grav(3) = q(1)*q(1) - q(2)*q(2) - q(3)*q(3) + q(4)*q(4);

% Calculates the yaw, pitch, and roll
ypr(1) = atan2(2*q(2)*q(3) - 2*q(1)*q(4), 2*q(1)*q(1) + 2*q(2)*q(2) - 1);
ypr(2) = atan(grav(1) / sqrt(grav(2)*grav(2) + grav(3)*grav(3)));
ypr(3) = atan(grav(2) / sqrt(grav(1)*grav(1) + grav(3)*grav(3)));