function [error,G] = EuclideanNorm(X, KC, pointsd)
%% ============================Euclidean Norm==============================
% RoboHAZMAT: Senior Design Project
% Motion Control Team
% Gerardo Bledt
% October 20, 2014
%
% The error function that uses the Euclidean norm to minimize the distance
% between the desired and actual points.

% Initialize error
error = 0;

% Pull out relevant parameters
KC = RotateKinematicChain(KC, X);
pointsi = KC.points.kPG;
weightings = KC.optimization.weightings;

% Calculate weighted error of the difference between desired and actual
for i = 1:length(weightings)
    error = error + ...
        weightings(i)*((pointsi(1,i) - pointsd(1,i))^2 + ...
        (pointsi(2,i) - pointsd(2,i))^2 + (pointsi(3,i) - pointsd(3,i))^2);
end
% error = sqrt(error);

% Gradient is calculated if it is required
if nargout > 1
    G = KCGradient(X, KC, pointsd);
end