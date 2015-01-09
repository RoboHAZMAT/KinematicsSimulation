function KC = RotateKinematicChain(KC, X)
%% ========================Rotate Kinematic Chain==========================
% RoboHAZMAT: Senior Design Project
% Motion Control Team
% Gerardo Bledt
% October 20, 2014
%
% Rotates a Kinematic chain specified by the X vector of angle rotations.
% Absolute rotation from the initial point.

% Add the rotation to the DOF
KC.DHParams.thetas = (KC.thetas.thi + X);

% Homogeneous transformations
KC.DHParams.H = DHTransforms(KC.DHParams);

% Transform each point in the global frame
for i = 1:size(KC.points.kP,2)
    KC.points.kPG(:,i) = ...
        KC.DHParams.HGo*KC.DHParams.H(:,:,KC.points.frames(i))*KC.points.kP(:,i);
end
KC.points.pG = KC.points.kPG(:,1:KC.DOF);