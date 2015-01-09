function traj = TrajectoriesRoboHAZMAT(i, traj)

if (traj.traj == 1)
    if (i == 0)
        traj.velocity = 8;  % Set velocity
        traj.runs = 360; % Number of runs
        traj.noise = 0; % No noise
    else
        % Creates the vertical figure 8 trajectory
        traj.point(1,1) = 0.05*cosd(2*i + 180) + 0.35;
        traj.point(2,1) = 0.2*sind(2*i) - 0.15;
        traj.point(3,1) = 0.3*cosd(1*i) + 0.3;
    end
    
elseif (traj.traj == 2)
    if (i == 0)
        traj.velocity = 8;  % Set velocity
        traj.runs = 360; % Number of runs
        traj.noise = 1; % Adds noise
    else
        % Creates a vaguely circular noisy trajectory
        traj.point(1,1) = 0.05*cosd(2*i) + 0.35 + noiseCalc;
        traj.point(2,1) = -0.2*sind(i + 210) - 0.15 + noiseCalc;
        traj.point(3,1) = 0.2*cosd(i) + 0.3 + noiseCalc;
    end
end