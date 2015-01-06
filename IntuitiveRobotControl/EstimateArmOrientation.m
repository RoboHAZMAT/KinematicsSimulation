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
linkRot = zeros(3,3);

% Third link is dependant on the second link
q(3,:) = q(2,:);
reset(3) = 0;
psi(3) = psi(2);

% Rotate each of the links
for i = 1:size(linkRot, 1)
    % Rotate link by quaternion
    linkRot(i,:) = quaternionRotate(q(i,:),link(i,:));
    
    % Zero the rotated link to the specified zero
    [linkRot(i,:), psi(i)] = ZeroYaw(linkRot(i,:), psi(i), reset(i));
end