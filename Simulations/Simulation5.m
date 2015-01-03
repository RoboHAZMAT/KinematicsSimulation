function Robot = Simulation5(Robot)

% Sets up the kinematics
KC = Robot.KinematicChains.RMK;
pointsd = zeros(4, size(KC.points.p,2));

% History vectors for trajectories
xHist = zeros(1,360);
yHist = zeros(1,360);
zHist = zeros(1,360);
xHistT = zeros(1,360);
yHistT = zeros(1,360);
zHistT = zeros(1,360);

% Runs the trajectory multiple times
for i = 1:7:360*3
    % Creates the figure 8 trajectory
    x = 0.05*cosd(2*i + 180) + 0.35;
    y = 0.2*sind(2*i) - 0.15;
    z = 0.3*cosd(1*i) + 0.3;
    
    xHist(i) = x;
    yHist(i) = y;
    zHist(i) = z;
    
    % Adds noise to trajectory and carries out inverse
    % kinematics on the noisy trajectory
    pointsd(:,size(KC.points.p,2)) = [x + noiseCalc; ...
        y + noiseCalc; z + noiseCalc;1];
    X = InverseKinematicOptimization(Robot,'RMK',pointsd);
    
    % Rotates and plots the right arm to optimized value
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
