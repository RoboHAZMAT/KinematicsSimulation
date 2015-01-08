function [x, y, z] = TrajectoriesMechatronicArm(i, traj, trajDur)

if (nargin <= 2)
    trajDur = 0;
end

if (traj == 1)
    % Creates the vertical figure 8 trajectory
    x = 0.0*cosd(2*i + 180) + 0.175;
    y = 0.05*sind(2*i) + 0;
    z = 0.05*cosd(1*i) + 0.1;
    
elseif (traj == 2)
    % Creates the horizontal figure 8 trajectory
    x = 0.05*cosd(i) + 0.15;
    y = 0.05*sind(2*i) + 0;
    z = 0.00*cosd(1*i) + 0.1;
    
elseif (traj == 3)
    % Object pickup and move
    
    if (i < trajDur/2)
        % Go to red marker
        x = 0.12 + 0.01*2*i/trajDur;
        y = 0.0975*2*i/trajDur;
        z = 0.16;
        
    elseif (i < trajDur)
        % Go to red marker
        x = 0.13;
        y = 0.0975;
        z = 0.16 - 0.085*i/trajDur;
        
    elseif (i < 2*trajDur)
        % Raise from red marker
        x = 0.13;
        y = 0.0975;
        z = 0.075 + 0.085*mod(i,trajDur)/(trajDur);
        
    elseif (i < 3*trajDur)
        % Go to black marker from red marker
        x = 0.13;
        y = 0.0975 - 0.1775*mod(i,trajDur)/(trajDur);
        z = 0.16;
        
    elseif (i < 4*trajDur)
        % Lower onto black marker
        x = 0.13;
        y = -0.08;
        z = 0.16 - 0.085*mod(i,trajDur)/(trajDur);
        
    elseif (i < 5*trajDur)
        % Raise from black marker
        x = 0.13;
        y = -0.08;
        z = 0.075 + 0.085*mod(i,trajDur)/(trajDur);
        
    elseif (i < 6*trajDur)
        % Go to center from black marker
        x = 0.13;
        y = -0.08 + 0.08*mod(i,trajDur)/(trajDur);
        z = 0.16;
        
    elseif (i < 7*trajDur)
        % Go to black marker from center
        x = 0.13;
        y = 0 - 0.08*mod(i,trajDur)/(trajDur);
        z = 0.16;
        
    elseif (i < 8*trajDur)
        % Lower onto black marker
        x = 0.13;
        y = -0.08;
        z = 0.16 - 0.085*mod(i,trajDur)/(trajDur);
        
    elseif (i < 9*trajDur)
        % Raise from black marker
        x = 0.13;
        y = -0.08;
        z = 0.075 + 0.085*mod(i,trajDur)/(trajDur);
        
    elseif (i < 10*trajDur)
        % Go to red marker from black marker
        x = 0.13 - 0.01*mod(i,trajDur)/(trajDur);
        y = -0.08 + 0.17*mod(i,trajDur)/(trajDur);
        z = 0.16;
        
    elseif (i < 11*trajDur)
        % Lower onto red marker
        x = 0.12;
        y = 0.09;
        z = 0.16 - 0.085*mod(i,trajDur)/(trajDur);
        
    elseif (i < 12*trajDur)
        % Raise from red marker
        x = 0.12;
        y = 0.09;
        z = 0.075 + 0.085*mod(i,trajDur)/(trajDur);
        
    elseif (i < 13*trajDur)
        % Go to center from black marker
        x = 0.12;
        y = 0.09 - 0.09*mod(i,trajDur)/(trajDur);
        z = 0.16;
    end
    
end