function Robot = Simulation4(Robot)

% Sets up the kinematics
KC = Robot.KinematicChains.RMK;
pointsd = zeros(4, size(KC.points.p,2));

% History vectors for the trajectories
xHist = zeros(1,360);
yHist = zeros(1,360);
zHist = zeros(1,360);
xHistT = zeros(1,360);
yHistT = zeros(1,360);
zHistT = zeros(1,360);

% Runs the loop a given number of times
for i = 1:5:360
    % Creates a vaguely circular noisy trajectory
    x = 0.05*cosd(2*i + 180) + 0.35 + noiseCalc;
    y = 0.2*sind(i+30) - 0.15 + noiseCalc;
    z = 0.2*cosd(i) + 0.3 + noiseCalc;
    xHist(i) = x;
    yHist(i) = y;
    zHist(i) = z;
    
    % Inverse kinematics
    pointsd(:,size(KC.points.p,2)) = [x;y;z;1];
    X = InverseKinematicOptimization(Robot,'RMK',pointsd);
    
    % Rotates and plots the Robot object
    KC = RotateKinematicChain(KC,X);
    Robot.KinematicChains.RMK = KC;
    RobotPlot(Robot);
    
    % Plots the ghost trajectories
    xHistT(i) = KC.points.pG(1,6);
    yHistT(i) = KC.points.pG(2,6);
    zHistT(i) = KC.points.pG(3,6);
    
    plot3(xHist,yHist,zHist,'.','color','green');
    plot3(xHistT,yHistT,zHistT,'.','color','red');
    drawnow;
end