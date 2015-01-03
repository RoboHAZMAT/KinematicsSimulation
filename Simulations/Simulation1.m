function Robot = Simulation1(Robot)
%% Inverse kinematics with user inputting points

% Position defined inverse kinematics
fprintf('\nChoose Kinematic Chain to move.\n');
fprintf('Options are:\n');
fields = fieldnames(Robot.KinematicChains);
for i = 1:length(fields)
    fprintf([' ',fields{i},'\n']);
end

% User input of kinematic chain
KCstring = input(' > ','s');
valid = false;

% Get the kinematic chain
for i = 1:length(fields)
    if (strcmpi(KCstring, fields{i}))
        valid = true;
        KCname = fields{i};
        KC = Robot.KinematicChains.(fields{i});
    end
end

% If the robot has the kinematic chain, continue
if (valid)
    fprintf([KCstring,' was selected\n']);
    fprintf('\nChoose (x, y, z) location to move the gripper to.\n');
    
    % Input an x, y, z location to move the kinematic chain to
    x = input(' x = ');
    y = input(' y = ');
    z = input(' z = ');
    
    % Inverse kinematics
    pointsd = zeros(4, size(KC.points.p,2));
    pointsd(:,size(KC.points.p,2)) = [x;y;z;1];
    X = InverseKinematicOptimization(Robot,KCname,pointsd);
    
    % Rotates and plots the kinematic chain
    KC = RotateKinematicChain(KC,X);
    Robot.KinematicChains.(KCname) = KC;
    RobotPlot(Robot);
else
    fprintf(['\nInvalid Kinematic Chain name. Please choose ',...
        'from the list\n']);
end