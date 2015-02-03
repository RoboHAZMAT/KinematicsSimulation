function [linkRot, psi] = EstimateHeadOrientation(link, q, reset, psi)
%% ======================Estimate Head Orientation=========================
% RoboHAZMAT: Senior Design Project
% Motion Control Team
% Gerardo Bledt
% January 2, 2015
%
% Rotates a set of links specifying the orientation of the arm from the
% sensed IMU quaternion orientations.

% Initialize the link rotations
linkRot = zeros(2,3);
q(2,:) = q(1,:);
reset(2) = 0;

% Rotate each of the links
for i = 1:size(linkRot, 1)
    % Rotate link by quaternion
    linkRot(i,:) = QuaternionRotate(q(i,:),link(i,:));
    
    % Zero the rotated link to the specified zero
    [linkRot(i,:), psi(i)] = ZeroYaw(linkRot(i,:), psi(i), reset(i));
end