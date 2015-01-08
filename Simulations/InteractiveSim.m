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
%   ** 6: Uses two IMU sensors, one on the wrist and one in the middle of
%         the upper arm to estimate arm orientations. The estimated arm
%         orientations are used to calculate desired elbow, wrist, and hand
%         locations in 3D. An inverse kinematics optimization is run to
%         calculate the optimal theta angles for the simulated robot
%         kinematic chains to mimic the user's arm.
%
%   ** 7: Uses Serial communication to talk to the Mechatronic Arm through
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
        clc;
        fprintf('====================================\n\n');
        fprintf('     Interactive Simulation: %i\n\n', mode);
        fprintf('====================================\n\n');
        fprintf('       Running Simulation...\n');
        fprintf('          {Space to quit}\n');
        %% RoboHAZMAT {1 - 10} and Mechatronic Arm {11 - 20} Simulations
        if (mode == 1)
            % Inverse kinematics With User Inputting Points
            Robot = Simulation1(Robot);
            
        elseif (mode == 2)
            % Inverse Kinematic Tracing Trajectories
            Robot = Simulation2(Robot);
            
        elseif (mode == 3)
            % IMU Controlled Manipulators Simulation
            Robot = Simulation3(Robot);
            
        elseif (mode == 4)
            % IMU Controlled Robot Arm Simulation
            Robot = Simulation4(Robot);
            
        elseif (mode == 5)
            % Keyboard Controlled Robot Arms Simulation
            Robot = Simulation5(Robot);
            
        elseif (mode == 11)
            % Mechatronic Arm Inverse Kinematics Trajectory Simulation
            Robot = SimulationM1(Robot);
            
        elseif (mode == 12)
            % IMU controlled Mechatronic Arm Through Arduino (ypr)
            Robot = SimulationM2(Robot);
            
        elseif (mode == 13)
            % Mechatronic Arm Controlled Through Arduino
            Robot = SimulationM3(Robot);
        
        elseif (mode == 14)
            % Keyboard Controlled Mechatronic Arm Simulation
            Robot = SimulationM4(Robot);
            
        elseif (mode == 15)
            % IMU controlled Mechatronic Arm Through Arduino (IK)
            % Robot = SimulationM5(Robot);
            
Robot = Simulation1(Robot);
            
        else
            %% Incorrect mode input loop.
            % Asks the user to enter a supported simulation mode or quit.
            while (mode - 14 > 0 || mode < 0)
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