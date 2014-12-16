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
KC.DHTransforms.H = DHTransforms(KC.DHParams);

% Transform each point in the global frame
for i = 1:KC.DOF
    KC.points.pG(:,i) = ...
        KC.DHTransforms.HGo*KC.DHTransforms.H(:,:,i)*KC.points.p(:,i);
end