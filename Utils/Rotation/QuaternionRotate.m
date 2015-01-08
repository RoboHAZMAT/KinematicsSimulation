function rout = QuaternionRotate(q, r)
%% =========================Quaternion Rotation============================
% RoboHAZMAT: Senior Design Project
% Motion Control Team
% Gerardo Bledt
% December 15, 2015
%
% Rotate a vector by a given quaternion.

if(size(q,2) ~= 4), error('q must be 1-by-4'); end;
if(size(r,2) ~= 3), error('r must be M-by-3'); end;

% Normalize the quaternion
%qin = q./( sqrt(sum(q.^2,2)) * ones(1,4));
qin = q/norm(q);

% Form the roatation matrix from the quaternion
R = zeros(3,3);

R(1,1) = (1 - 2*q(3)^2-2*q(4)^2);
R(1,2) = 2*(q(2)*q(3) + q(1)*q(4));
R(1,3) = 2*(q(2)*q(4) - q(1)*q(3));

R(2,1) = 2*(q(2)*q(3) - q(1)*q(4));
R(2,2) = (1 - 2*q(2)^2-2*q(4)^2);
R(2,3) = 2*(q(3)*q(4) + q(1)*q(2));

R(3,1) = 2*(q(2)*q(4) + q(1)*q(3));
R(3,2) = 2*(q(3)*q(4) - q(1)*q(2));
R(3,3) = (1 - 2*q(2)^2-2*q(3)^2);

% Rotate the vector by matrix
rout = (R'*r')';