function X = optimize(robot, name, pts)
% Optimization function that takes in a robotic system, the name of the
% kinematic chain to move around, and the points to move the frame points
% to and uses fmincon to move the system around.


KC = EmptyKinematics();
fields = fieldnames(robot.KinematicChains);

for i = 1:length(fields)
    if strcmpi(fields(i),name)
        KC = robot.KinematicChains.(fields{i});
    end
end
X0 = KC.DHParams.thetas - KC.thetas.thi;

% opts = optimset('TolFun',5e-2);
% X = fminsearch(@(X)euclideanNorm(X,KC,pts),X0,opts);
options = optimset('Algorithm','interior-point','GradObj','on','Hessian',...
    'user-supplied','Hessfcn',@(X, lambda)KCHessian(X, lambda, KC, pts),...
    'Display','off','TolFun',1e-2,'DiffMinChange',2/180*pi);
% [X,fval,exitflag,output,lambda,grad,hessian] = fmincon(@(X)euclideanNorm(X,KC,pts),X0,[],[],[],[],...
%     KC.bounds.lb,KC.bounds.ub,[],options);
% options = optimset('Algorithm','interior-point','GradObj','on','Display','off',...
%     'TolFun',1e-2,'DiffMaxChange',10,'DiffMinChange',2/180*pi);
X = fmincon(@(X)euclideanNorm(X,KC,pts),X0,[],[],[],[],...
    KC.bounds.lb,KC.bounds.ub,[],options);