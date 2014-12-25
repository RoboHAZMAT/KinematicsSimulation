function [error,G] = euclideanNorm(X, KC, pointsd)
% The error function that uses the Euclidean norm to minimize the distance
% between the desired and actual points.
error = 0;
KC = RotateKinematicChain(KC, X);
pointsi = KC.points.pG;
weightings = KC.weightings;


for i = 1:length(weightings)
    error = error + ...
        weightings(i)*((pointsi(1,i) - pointsd(1,i))^2 + ...
        (pointsi(2,i) - pointsd(2,i))^2 + (pointsi(3,i) - pointsd(3,i))^2);
end
%error = sqrt(error);
if nargout > 1
    G = KCGradient(X, KC, pointsd);
end