function Robot = InteractiveSim(Robot, mode)
%% ========================Interactive Simulation==========================
% RoboHAZMAT: Senior Design Project
% Motion Control Team
% Gerardo Bledt
% December 15, 2014
%
% Runs an interactive Simulation for the robotic system chosen. When
% running the 'RobotSim.m' function, add an input argument to run one of
% the simulations. Currently there are 6 supported options that are
% numbered 1 through 6.
%
%   These modes are:
%
%      1: Inverse kinematics demonstration With the RoboHAZMAT robotic
%         system. Allows the user to input the name of one of the kinematic
%         chains that make up the robot and an x, y, z coordinate that the
%         optimization script will attempt to move the end point to.
%
%    * 2: IMU usage for the RoboHAZMAT system that moves the forearm of the
%         robot according to the yaw, pitch, and roll angles that the IMU
%         is at. Moves the Right manipulator of the robot.
%
%    * 3: IMU usage for the RoboHAZMAT system that moves the forearm of the
%         robot according to the yaw, pitch, and roll angles that the IMU
%         is at. Moves the Left manipulator of the robot.
%
%      4: Demonstration of the RoboHAZMAT robot's ability to track a given
%         path with noise. Draws a circular trajectory and has the robot's
%         Right manipulator trace it back.
%
%      5: Demonstration of the RoboHAZMAT robot's ability to track a given
%         path quickly with low latency in the inverse kinematics. Draws a
%         figure eight trajectory and has the robot's Right manipulator
%         trace it back in real-time.
%
%   ** 6: Uses Serial communication to talk to the Mechatronic Arm through
%         the Arduino and takes in information from the Serial port
%         connected to the IMU reading program. Uses the incoming data to
%         control the arm as well as display a simulated version of the arm
%         in a MATLAB figure.
%
%   * Signifies connection required to the IMU running the IMUArm.ino
%     program on arduino.
%
%  ** Signifies connection required to the IMU running the IMUArm.ino
%     program on arduino as well as an Arduino connected over Serial.
%
%
%  To Do:
%    - Add a simulation involving 2 IMU sensors and a quaternion
%        representation of the human arm configuration in vectors
%    - Use that representation with inverse kinematics to find the optimal
%        configuration necessary for the robot arm

attempt = 0;
interactiveSim = input('Begin Interactive Simulation? [Y/N]: ','s');

% Allow for 3 attempts to enter the interactive simulation mode by entering
% in [Y/y/ ] or to voluntarily quit by entering [N/n]
while (~strcmpi(interactiveSim,'yes') && ~strcmpi(interactiveSim,'no')...
        && ~strcmpi(interactiveSim,'y') && ~strcmpi(interactiveSim,'n')...
        && ~strcmpi(interactiveSim,''))
    attempt = attempt + 1;
    if (attempt == 3)
        fprintf('\nFinal attempt...');
    elseif (attempt == 4)
        fprintf(['\nUnrecognized Command. Too many failed attempts.',...
            '\nQuitting Simulation...']);
        break;
    end
    fprintf('\nUnrecognized command, please enter [Y/N]\n');
    interactiveSim = input('Begin Interactive Simulation? [Y/N]: ','s');
end

% If the user chose to enter the interactive simulation mode, begin
if (strcmpi(interactiveSim,'yes') || strcmpi(interactiveSim,'y'))
    fprintf('\nBeginning Interactive Simulation...\n');
    while (strcmpi(interactiveSim,'yes') || strcmpi(interactiveSim,'y'))
        if (mode == 1)
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
                X = optimize(Robot,KCname,pointsd);
                
                % Rotates and plots the kinematic chain
                KC = RotateKinematicChain(KC,X);
                Robot.KinematicChains.(KCname) = KC;
                RobotPlot(Robot);
            else
                fprintf(['\nInvalid Kinematic Chain name. Please choose ',...
                    'from the list\n']);
            end
            
        elseif (mode == 2)
            %% IMU controlled Right manipulator simulation
            
            % Sets up COM ports
            IMUCOM = SetupCOM;
            
            % Right Manipulator simulation controlled by IMU
            serialObjIMU = SetupIMUSerial(IMUCOM(1,:));
            
            KC = Robot.KinematicChains.RMK;
            while (mode == 2)
                % Reads the IMU
                [yaw, pitch, roll, readingIMU] = ReadIMU(serialObjIMU);
                if (~isnan(readingIMU))
                    % Rotation vector
                    X = zeros(6,1);
                    X(2,1) = (-(yaw))/180*pi;
                    X(4,1) = (-(pitch + 90))/180*pi;
                    X(5,1) = (-(roll))/180*pi;
                    X(6,1) = (90)/180*pi;
                    
                    % Rotates and plots the kinematic chain
                    KC = RotateKinematicChain(KC,X);
                    Robot.KinematicChains.RMK = KC;
                    RobotPlot(Robot);
                    drawnow;
                end
            end
            
        elseif (mode == 3)
            %% IMU controlled Left manipulator simulation
            
            % Sets up the COM ports
            IMUCOM = SetupCOM;
            
            % Left Manipulator simulation controlled by IMU
            serialObjIMU = SetupIMUSerial(IMUCOM(1,:));
            
            KC = Robot.KinematicChains.LMK;
            %calibrateIMU();
            while (mode == 3)
                % Reads the IMU
                [yaw, pitch, roll, readingIMU] = ReadIMU(serialObjIMU);
                if (~isnan(readingIMU))
                    % Rotation vector
                    X = zeros(6,1);
                    X(2,1) = (-(yaw))/180*pi;
                    X(4,1) = (-(pitch + 90))/180*pi;
                    X(5,1) = (-(roll))/180*pi;
                    X(6,1) = (90)/180*pi;
                    
                    % Rotates and plots the kinematic chain
                    KC = RotateKinematicChain(KC,X);
                    Robot.KinematicChains.LMK = KC;
                    RobotPlot(Robot);
                    drawnow;
                end
            end
        elseif (mode == 4)
            %% Inverse kinematic tracing of noisy trajectory
            
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
                X = optimize(Robot,'RMK',pointsd);
                
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
            
        elseif (mode == 5)
            %% Figure 8 inverse kinematics tracing in real-time
            
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
                X = optimize(Robot,'RMK',pointsd);
                
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
            
        elseif (mode == 6)
            %% IMU controlled Mechatronic Arm through Arduino
            
            % Setup COM ports
            [IMUCOM, motorControlCOM] = SetupCOM;
            
            % Setup communication with IMU and Arm
            serialObjIMU = SetupIMUSerial(IMUCOM(1,:));
            [serialMotorControl, motor] = ...
                SetupMotorControlSerial(motorControlCOM);
            
            % Constrain unused motors to the given values
            serialMotorControl.servoWrite(motor.elbowPitch, round(150));
            serialMotorControl.servoWrite(motor.wristPitch, round(30));
            %serialMotorControl.servoWrite(motor.wristRoll, round(110));
            
            % Get the kinematic chain
            KC = Robot.KinematicChains.MAK;
            X = zeros(5,1);
            
            % Constant running while loop:
            % To quit: 'Ctrl+C' (May try to find a better way)
            while (mode == 6)
                
                % Read the IMU sensor
                [yaw, pitch, roll, readingIMU] = ReadIMU(serialObjIMU);
                
                % If a reading is obtained, rotate simulation and arm
                if (~isnan(readingIMU))
                    % Rotates the base yaw, base pitch, and wrist roll
                    serialMotorControl.servoWrite(motor.baseYaw, ...
                        round(yaw));
                    serialMotorControl.servoWrite(motor.basePitch, ...
                        round(pitch));
                    serialMotorControl.servoWrite(motor.wristRoll, ...
                        round(180-roll));
                    
                    % Rotate and plot the simulated Mechatronic Arm
                    X(1,1) = (-(yaw))/180*pi;
                    X(2,1) = (-pitch + 90)/180*pi;
                    X(3,1) = (60)/180*pi;
                    X(4,1) = (30)/180*pi;
                    X(5,1) = (-(roll))/180*pi;
                    KC = RotateKinematicChain(KC,X);
                    Robot.KinematicChains.MAK = KC;
                    RobotPlot(Robot);
                    drawnow;
                end
            end
        elseif (mode == 7)
            %% IMU controlled Mechatronic Arm through Arduino
            IMUCOM = SetupCOM;
            serialObjIMU(1) = SetupIMUSerial(IMUCOM(1,:));
            serialObjIMU(2) = SetupIMUSerial(IMUCOM(2,:));
            
            KC = Robot.KinematicChains.RMK;
            KC.weightings(4) = 300;
            KC.weightings(5) = 100;
            KC.weightings(6) = 200;
            
            shoulder = [0,-0.179,0.371];
            link(1,:) = [.279,0,0];
            link(2,:) = [0.257,0,0];
            link(3,:) = [0,0,-0.076];
            
            linkRot = zeros(2,3);
            q = zeros(2,4);
            pointsd = zeros(4, size(Robot.KinematicChains.RMK.points.p,2));
            
            psi = [0, 0];
            
            while (1)
                [qw(1), qx(1), qy(1), qz(1), reset(1)] = ReadIMUQuaternion(serialObjIMU(1));
                [qw(2), qx(2), qy(2), qz(2), reset(2)] = ReadIMUQuaternion(serialObjIMU(2));
                if (~isnan(qw(1)) && ~isnan(qw(2)))
                    
                    for i = 1:3
                        if i == 3
                            q(i,:) = [qw(2), qx(2), qy(2), qz(2)];
                            linkRot(i,:) = quatrotate(q(i,:),link(i,:));
                            linkRot(i,:) = zeroYaw(linkRot(i,:), psi(2));
                        else
                            q(i,:) = [qw(i), qx(i), qy(i), qz(i)];
                            linkRot(i,:) = quatrotate(q(i,:),link(i,:));
                            if (reset(i) == 1)
                                psi(i) = yawOffset(linkRot(i,:));
                            end
                            linkRot(i,:) = zeroYaw(linkRot(i,:), psi(i));
                        end
                    end
                    
                    elbow = [shoulder(1,1) + linkRot(1,1),...
                        shoulder(1,2) + linkRot(1,2),shoulder(1,3) + linkRot(1,3)];
                    wrist = [elbow(1,1) + linkRot(2,1),...
                        elbow(1,2) + linkRot(2,2),elbow(1,3) + linkRot(2,3)];
                    hand = [wrist(1,1) + linkRot(3,1),...
                        wrist(1,2) + linkRot(3,2),wrist(1,3) + linkRot(3,3)];
                    
                    pointsd(:,3) = [elbow';1];
                    pointsd(:,4) = [elbow';1];
                    pointsd(:,5) = [wrist';1];
                    pointsd(:,6) = [hand';1];
                    
                    X = optimize(Robot,'RMK',pointsd);
                    
                    % Rotates and plots the right arm to optimized value
                    KC = RotateKinematicChain(KC,X);
                    Robot.KinematicChains.RMK = KC;
                    RobotPlot(Robot);
                    
                    LW = 3; MS = 15;
                    plot3([shoulder(1,1),elbow(1,1),wrist(1,1),hand(1,1)],...
                        [shoulder(1,2),elbow(1,2),wrist(1,2),hand(1,2)],...
                        [shoulder(1,3),elbow(1,3),wrist(1,3),hand(1,3)],...
                        'LineWidth',LW,'Color',[0 0 0],'Marker','.',...
                        'MarkerEdgeColor',[1 0 0],'MarkerSize',MS);
                    drawnow;
                end
            end
        else
            
            % Incorrect mode input loop. Asks the user to enter a supported
            % simulation mode or to quit.
            while (mode - 6 > 0 || mode < 0)
                fprintf(['\nMode %i is not supported.\n Choose 1 - 6,',...
                    ' q to quit:\n'],mode);
                modeString = input(' > ','s');
                if (strcmpi(modeString,'q'))
                    fprintf('\nQuitting Simulation...\n');
                    break;
                end
                mode = str2double(modeString);
            end
        end
        
        % Incorrect continue input loop. Asks the user to continue or not.
        % After 3 incorrect tries, the program will quit.
        attempt = 0;
        interactiveSim = 'a';
        while (~strcmpi(interactiveSim,'yes') && ...
                ~strcmpi(interactiveSim,'no') && ...
                ~strcmpi(interactiveSim,'y') && ...
                ~strcmpi(interactiveSim,'n') && ...
                ~strcmpi(interactiveSim,''))
            attempt = attempt + 1;
            if (attempt == 3)
                fprintf('\nFinal attempt...');
            elseif (attempt == 4)
                fprintf(['\nUnrecognized Command. Too many failed ',...
                    'attempts.\nQuitting Simulation...']);
                interactiveSim = 'n';
                break;
            end
            if (attempt > 1)
                fprintf('\nUnrecognized command, please enter [Y/N]\n');
            end
            interactiveSim = input('\nContinue Interactive Simulation? [Y/N]: ','s');
        end
    end
    % Otherwise, quit the simulation
elseif (strcmpi(interactiveSim,'no') || strcmpi(interactiveSim,'n')...
        || strcmpi(interactiveSim,''))
    fprintf('\nQuitting Simulation...\n');
end