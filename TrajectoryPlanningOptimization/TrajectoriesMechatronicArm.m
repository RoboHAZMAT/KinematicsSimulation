function [x, y, z] = TrajectoriesMechatronicArm(i, traj, serialMotorControl, motor)

if (traj == 1)
    % Creates the vertical figure 8 trajectory
    x = 0.0*cosd(2*i + 180) + 0.1;
    y = 0.05*sind(2*i) + 0;
    z = 0.05*cosd(1*i) + 0.05;
    
elseif (traj == 2)
    % Creates the horizontal figure 8 trajectory
    x = 0.05*cosd(i) + 0.15;
    y = 0.05*sind(2*i) + 0;
    z = 0.00*cosd(1*i) + 0.1;
    
elseif (traj == 3)
    n = 60;
    
    if (mod(i-1, n) == 0)
        if (i-1 == n)
            serialMotorControl.servoWrite(motor.gripper, 180);
            pause(0.5);
        elseif (i-1 == 4*n)
            serialMotorControl.servoWrite(motor.gripper, 0);
            pause(0.5);
        elseif (i-1 == 8*n)
            serialMotorControl.servoWrite(motor.gripper, 180);
            pause(0.5);
        elseif (i-1 == 11*n)
            serialMotorControl.servoWrite(motor.gripper, 0);
            pause(0.5);
        end
    end
    
    if (i < n/2)
        % Go to red marker
        x = 0.12 + 0.01*2*i/n;
        y = 0.0975*2*i/n;
        z = 0.16;
        
    elseif (i < n)
        % Go to red marker
        x = 0.13;
        y = 0.0975;
        z = 0.16 - 0.085*i/n;
        
    elseif (i < 2*n)
        % Raise from red marker
        x = 0.13;
        y = 0.0975;
        z = 0.075 + 0.085*mod(i,n)/(n);
        
    elseif (i < 3*n)
        % Go to black marker from red marker
        x = 0.13;
        y = 0.0975 - 0.1775*mod(i,n)/(n);
        z = 0.16;
        
    elseif (i < 4*n)
        % Lower onto black marker
        x = 0.13;
        y = -0.08;
        z = 0.16 - 0.085*mod(i,n)/(n);
        
    elseif (i < 5*n)
        % Raise from black marker
        x = 0.13;
        y = -0.08;
        z = 0.075 + 0.085*mod(i,n)/(n);
        
    elseif (i < 6*n)
        % Go to center from black marker
        x = 0.13;
        y = -0.08 + 0.08*mod(i,n)/(n);
        z = 0.16;
        
    elseif (i < 7*n)
        % Go to black marker from center
        x = 0.13;
        y = 0 - 0.08*mod(i,n)/(n);
        z = 0.16;
        
    elseif (i < 8*n)
        % Lower onto black marker
        x = 0.13;
        y = -0.08;
        z = 0.16 - 0.085*mod(i,n)/(n);
        
    elseif (i < 9*n)
        % Raise from black marker
        x = 0.13;
        y = -0.08;
        z = 0.075 + 0.085*mod(i,n)/(n);
        
    elseif (i < 10*n)
        % Go to red marker from black marker
        x = 0.13 - 0.01*mod(i,n)/(n);
        y = -0.08 + 0.17*mod(i,n)/(n);
        z = 0.16;
        
    elseif (i < 11*n)
        % Lower onto red marker
        x = 0.12;
        y = 0.09;
        z = 0.16 - 0.085*mod(i,n)/(n);
        
    elseif (i < 12*n)
        % Raise from red marker
        x = 0.12;
        y = 0.09;
        z = 0.075 + 0.085*mod(i,n)/(n);
        
    elseif (i < 13*n)
        % Go to center from black marker
        x = 0.12;
        y = 0.09 - 0.09*mod(i,n)/(n);
        z = 0.16;
    end
    
end