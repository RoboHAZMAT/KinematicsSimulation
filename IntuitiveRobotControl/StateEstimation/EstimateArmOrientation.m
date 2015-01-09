function [linkRot, psi] = EstimateArmOrientation(link, q, reset, psi)
%% =======================Estimate Arm Orientation=========================
% RoboHAZMAT: Senior Design Project
% Motion Control Team
% Gerardo Bledt
% January 2, 2015
%
% Rotates a set of links specifying the orientation of the arm from the
% sensed IMU quaternion orientations.

% Initialize the link rotations
linkRot = zeros(4,3);

% Shift dependant links
q(3,:) = q(2,:);
q(4,:) = q(2,:);
q(2,:) = q(1,:);
reset(3) = reset(2);
reset(2) = 0;
psi(3) = psi(2);
psi(4) = psi(2);
psi(2) = psi(1);
reset(4) = 0;

% Rotate each of the links
for i = 1:size(linkRot, 1)
    % Rotate link by quaternion
    linkRot(i,:) = QuaternionRotate(q(i,:),link(i,:));
    
    % Zero the rotated link to the specified zero
    [linkRot(i,:), psi(i)] = ZeroYaw(linkRot(i,:), psi(i), reset(i));
end
psi(2) = psi(3);