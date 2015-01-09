function traj = TrajectoriesMechatronicArm(i, traj)

if (traj.traj == 1)
    if (i ==0)
        traj.velocity = 5;  % Set velocity
        traj.runs = (360 - 1); % Number of runs
    else
        % Creates the vertical figure 8 trajectory
        traj.point(1,1) = 0.0*cosd(2*i + 180) + 0.175;
        traj.point(2,1) = 0.05*sind(2*i) + 0;
        traj.point(3,1) = 0.05*cosd(1*i) + 0.1;
    end
    
elseif (traj.traj == 2)
    if (i ==0)
        traj.velocity = 5;  % Set velocity
        traj.runs = (360 - 1); % Number of runs
    else
        % Creates the horizontal figure 8 trajectory
        traj.point(1,1) = 0.05*cosd(i) + 0.15;
        traj.point(2,1) = 0.05*sind(2*i) + 0;
        traj.point(3,1) = 0.00*cosd(1*i) + 0.1;
    end
    
elseif(traj.traj == 3)
    if (i == 0)
        traj.velocity = 1; % Set velocity: {1,..,.5}
        traj.runs = 14;    % 15 for a full cycle
        traj.point = [0;0;0];
    else
        % Time delay between steps
        delay = traj.delay;
        grabDelay = 0.3;
        
        % Max and mins for trajectory
        zgrab = 0.075; zup = 0.15;
        
        grabL = [0.145;0.08;zgrab];
        upL = [0.14;0.08;zup];
        placeL = [0.143;0.082;0.105];
        
        grabR = [0.14;-0.095;zgrab];
        upR = [0.14;-0.095;zup];
        placeR = [0.135;-0.09;0.105];
    end
    
    % Steps
    if (i == 1 || i == 8)      % Center the gripper
        traj.serialMotorControl.servoWrite(traj.motor.gripper, 0);
        pause(delay); traj.point = [mean([upR(1);upL(1)]); 0; zup];
    elseif (i == 2 || i == 12)  % Go above left side
        pause(delay); traj.point = upL;
    elseif (i == 3)  % Go onto left side
        pause(delay); traj.point = grabL;
    elseif (i == 4)  % Close gripper and go above left side
        pause(delay); traj.serialMotorControl.servoWrite(traj.motor.gripper, 180);
        pause(grabDelay); traj.point = upL;
    elseif (i == 5 || i == 9)  % Go above right side
        pause(delay); traj.point = upR;
    elseif (i == 6)  % Go onto right side
        pause(delay); traj.point = placeR;
    elseif (i == 7)  % Open gripper and go above right side
        pause(delay); traj.serialMotorControl.servoWrite(traj.motor.gripper, 0);
        pause(grabDelay); traj.point = upR;
    elseif (i == 10)  % Go onto right side
        pause(delay); traj.point = grabR;
    elseif (i == 11)  % Close gripper and go above right side
        pause(delay); traj.serialMotorControl.servoWrite(traj.motor.gripper, 180);
        pause(grabDelay); traj.point = upR;
    elseif (i == 13) % Go onto left side
        pause(delay); traj.point = placeL;
    elseif (i == 14) % Open gripper and go above left side
        pause(delay); traj.serialMotorControl.servoWrite(traj.motor.gripper, 0);
        pause(grabDelay); traj.point = upL;
    end
end