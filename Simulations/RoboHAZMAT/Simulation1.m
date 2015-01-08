function Robot = Simulation1(Robot)
%% ============Inverse kinematics With User Inputting Points===============

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
    if (strcmp(KCstring, fields{i}))
        valid = true;
        KCname = fields{i};
        KC = Robot.KinematicChains.(fields{i});
    end
end

controlPoint = 6;
KC.optimization.weightings(controlPoint) = 10;

% If the robot has the kinematic chain, continue
if (valid)
    fprintf([KCstring,' was selected\n']);
    fprintf('\nChoose (x, y, z) location to move the gripper to.\n');
    
    % Input an x, y, z location to move the kinematic chain to
    x = input(' x = ');
    y = input(' y = ');
    z = input(' z = ');
    
    % Inverse kinematics
    pointsd = [zeros(3, size(KC.points.kP,2));ones(1,size(KC.points.kP,2))];
    pointsd(:,controlPoint) = [x;y;z;1];
    X = InverseKinematicOptimization(KC, pointsd);
    
    % Rotates and plots the kinematic chain
    KC = RotateKinematicChain(KC,X);
    Robot.KinematicChains.(KCname) = KC;
    RobotPlot(Robot);
else
    fprintf(['\nInvalid Kinematic Chain name. Please choose ',...
        'from the list\n']);
end