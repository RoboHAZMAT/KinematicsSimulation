function [x, y, z] = TrajectoriesRoboHAZMAT(i, traj, trajDur)

if (nargin <= 2)
    trajDur = 0;
end

if (traj == 1)
    % Creates the vertical figure 8 trajectory
    x = 0.05*cosd(2*i + 180) + 0.35;
    y = 0.2*sind(2*i) - 0.15;
    z = 0.3*cosd(1*i) + 0.3;
    
elseif (traj == 2)
    % Creates a vaguely circular noisy trajectory
    x = 0.05*cosd(2*i) + 0.35 + noiseCalc;
    y = -0.2*sind(i + 210) - 0.15 + noiseCalc;
    z = 0.2*cosd(i) + 0.3 + noiseCalc;
    
end