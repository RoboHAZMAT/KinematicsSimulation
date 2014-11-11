function X = LeftManipulatorConstrainedOptimization(Xd, X0)
X0 = [0,0,0,0,0,0,0];
upperBounds = [pi,pi,pi,pi,pi,pi,pi,pi];
lowerBounds = [-pi,-pi,-pi,-pi,-pi,-pi,-pi,-pi];
options = optimset('Algorithm','interior-point');
X = fmincon(@errorFunc,X0,[],[],[],[],lowerBounds,upperBounds,[],options);