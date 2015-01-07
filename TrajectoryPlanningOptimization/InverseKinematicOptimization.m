function X = InverseKinematicOptimization(KC, pts)
% Optimization function that takes in a robotic system, the name of the
% kinematic chain to move around, and the points to move the frame points
% to and uses fmincon to move the system around.

% Sets the initial theta angles to their current values for faster
% minimization.
X0 = KC.DHParams.thetas - KC.thetas.thi;

% The options for the optimization fmincon. Also specifies the gradient and
% the Hessian function, although the Hessian is computationally heavy and
% may be best left unspecified
% options = optimset('Algorithm','interior-point','GradObj','on','Hessian',...
%     'user-supplied','Hessfcn',@(X, lambda)KCHessian(X, lambda, KC, pts),...
%     'Display','off','TolFun',1e-2,'DiffMinChange',2/180*pi);
options = optimset('Algorithm','interior-point','GradObj','on','Display',...
    'off','TolFun',1e-2,'DiffMaxChange',10,'DiffMinChange',2/180*pi);
warning('off','all');

% Call to the fmincon optimization function to minimize the Euclidean norm
% of several key points in the kinematic chain and their desired setpoints
X = fmincon(@(X)EuclideanNorm(X,KC,pts),X0,[],[],[],[],...
    KC.optimization.bounds.lb,KC.optimization.bounds.ub,[],options);