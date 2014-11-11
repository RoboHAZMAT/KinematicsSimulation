function error = errorFunc(X, kinematicChain, pointsd)
error = 0;

kinematicChain = RotateKinematicChain(kinematicChain, X);
pointsi = kinematicChain.points.pG;
weightings = kinematicChain.weightings;

for i = 1:length(weightings)
    error = error + ...
        weightings(i)*(norm(pointsi(:,i) - pointsd(:,i)))^2;
end

error = sqrt(error);