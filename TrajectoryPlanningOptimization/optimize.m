function X = optimize(robot, name, pts)

KC = EmptyKinematics();
fields = fieldnames(robot.KinematicChains);

for i = 1:length(fields)
    if strcmpi(fields(i),name)
        KC = robot.KinematicChains.(fields{i});
    end
end
X0 = KC.DHParams.thetas;
options = optimset('Algorithm','interior-point','Display','off',...
    'TolFun',1e-2);
X = fmincon(@(X)errorFunc(X,KC,pts),X0,[],[],[],[],...
    KC.bounds.lb,KC.bounds.ub,[],options);