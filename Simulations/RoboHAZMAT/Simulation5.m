function Robot = Simulation5(Robot)
%% =================IMU Controlled Robot Arm Simulation====================
% Simulation that allows the robot to be controlled by IMUs worn on the 
% arm. One IMU is worn on the upper arm and one IMU on the forearm. This
% allows for an accurate arm positioning estimation. With the estimate of
% the desired user arm, the robot joint motor angles is calculated through 
% inverse kinematics optimization. 
%
% - An extra feature allows the user to draw trajectories in 3D space using
% gestures. Trajectory tracks the arm's wrist point.

% Sets up the Keyboard Control
[RobotFigure, states] = SetupKeyboardControl(Robot, 2);

% Sets up the Serial communication with the IMUs
IMUCOM = SetupCOM;
serialObjIMU(1) = SetupIMUSerial(IMUCOM(1,:));
serialObjIMU(2) = SetupIMUSerial(IMUCOM(2,:));
% serialObjIMU(3) = SetupIMUSerial(IMUCOM(3,:));
% serialObjIMU(4) = SetupIMUSerial(IMUCOM(4,:));

% Specifies the arm and points to be controlled
% *** This is what must be tuned ***
% Right Arm Control Gains
KCR = RotateKinematicChain(Robot.KinematicChains.RMK,...
    [-pi/2;zeros(4,1);pi/2]);
KCR.optimization.weightings = [0;0;0;10;10;10;10;10];
% Left Arm Control Gains
% KCL = RotateKinematicChain(Robot.KinematicChains.RML, [zeros(5,1);pi/2]);
% KCL.optimization.weightings = [0;0;0;10;10;10;10;10];

% Sets up the estimated actual arm position
shoulderR = [0,-0.179,0.371];
% shoulderL = [0,0.179,0.371];
link(1,:) = [.279,0,0]; link(2,:) = [0,0,-0.076];
link(3,:) = [0.257,0,0]; link(4,:) = [0,0,-0.076];

% History vectors for the actual trajectory histories
trajBuffer = 100;
histTR = zeros(trajBuffer,3);
% histTL = zeros(trajectoryBuffer,3);

% Waits for the user to be ready to use and initializes the arm
ready = ReadyForUse(RobotFigure);
psiR = Reset(serialObjIMU(1:2), link, zeros(1,4));
% psiL = Reset(serialObjIMU(3:4), link, zeros(1,4));

% Constant running while loop
% 1. Gets the simulation state
% 2. Reads the IMU data from the sensors
% 3. Estimates the orientation of the arm links
% 4. Reconstructs the user's arm and desired points
% 5. Inverse Kinematic optimization to estimate joint angles
% 6. Rotate and plot the robot, human arm, and trajectory
while (ready && states.run)
    
    % Runs the loop through the size of the trajectory history buffer
    for i = 1:trajBuffer
        
        % 1. Gets simulation state
        states = guidata(RobotFigure); if (~states.run), break; end;
        
        % 2. Reads the IMU data from the sensors
        [qR(1,:), resetR(1)] = ReadIMUQuaternion(serialObjIMU(1));
        [qR(2,:), resetR(2)] = ReadIMUQuaternion(serialObjIMU(2));
        % [qL(1,:), resetL(1)] = ReadIMUQuaternion(serialObjIMU(3));
        % [qL(2,:), resetL(2)] = ReadIMUQuaternion(serialObjIMU(4));
        
        % 3. Estimates the orientation of the arm links
        [linkRRot, psiR] = ...
            EstimateArmOrientation(link, qR, resetR, psiR);
        % [linkLRot, psiL] = ...
        %     EstimateArmOrientation(link, qL, resetL, psiL);
        
        % 4. Reconstructs the user's arm and desired points
        pointsdR = ReconstructArm(shoulderR, linkRRot);
        % pointsdL = ReconstructArm(shoulderL, linkLRot);
        
        % 5. Inverse Kinematic optimization to estimate joint angles
        XR = InverseKinematicOptimization(KCR, pointsdR);
        % XL = InverseKinematicOptimization(KCL, pointsdL);
        
        % 6. Rotate and plot the robot, human arm, and trajectory
        KCR = RotateKinematicChain(KCR, XR);
        Robot.KinematicChains.RMK = KCR;
        % KCL = RotateKinematicChain(KCL, XL);
        % Robot.KinematicChains.LMK = KCL;
        RobotPlot(Robot);
        
        PlotHumanArm(shoulderR, pointsdR);
        % PlotHumanArm(shoulderL, pointsdL);
        histTR = ManageTrajectory(i, histTR, KCR, RobotFigure, states);
        % histTL = ManageTrajectory(i, histTL, KCL, RobotFigure, states);
        drawnow;
    end
    
    % If user requests to stop, check again to avoid cutting off Serial
    if (~states.run && ReadyForUse(RobotFigure))
        states.run = 1; guidata(RobotFigure, states);
        psiR = Reset(serialObjIMU(1:2), link, psiR);
        KCR = RotateKinematicChain(KCR, [-pi/2;zeros(5,1)]);
        % psiL = Reset(serialObjIMU(3:4), link, psiL);
        % KCL = RotateKinematicChain(KCL, [-pi/2;zeros(5,1)]);
    end;
end

% Close Serial communication
delete(instrfindall);
end


%%=============================Ready For Use===============================
% Makes sure that the user is ready to begin controlling the robotic arm.

function ready = ReadyForUse(RobotFigure)
% Wait for user to be ready or quit if not
states = guidata(RobotFigure);
states.begin = -1;
guidata(RobotFigure, states);

fprintf('Begin control? [Y/N]: ');
while (states.begin ~= 0  && states.begin ~= 1)
    states = guidata(RobotFigure);
    if (states.begin == 1)
        fprintf('Y\n\n   Robot ARM Control   \n\n');
        ready = true;
    elseif (states.begin == 0)
        fprintf('N\n');
        ready = false;
    end
    guidata(RobotFigure, states);
    pause(0.1);
end
end


%%=================================Reset===================================
% Resets the arm orientation and outputs the offset psi angle.

function psi = Reset(serialObjIMU, link, psi)

% Gets the two IMU readings from the sensors
q(1,:) = ReadIMUQuaternion(serialObjIMU(1));
q(2,:) = ReadIMUQuaternion(serialObjIMU(2));

% Forces a reset
reset = ones(1,2);

% 2. Estimates the orientation of the arm links
[~, psi] = ...
    EstimateArmOrientation(link, q, reset, psi);
end


%%===========================Manage Trajectory=============================
% Manages the history trajectory recording, plotting, and clearing.

function histT = ManageTrajectory(i, histT, KC, RobotFigure, states)

% If user requests to record trajectory, store current end point
if (states.recordTraj)
    histT(i,:) = KC.points.kPG(1:3,KC.DOF - 1)';
end

% Clears the trajectory history if requested
if (states.clearTraj)
    histT = zeros(size(histT,1),3);
    states.clearTraj = 0;
    guidata(RobotFigure, states);
end

% Plotting parameters
MS = 15;

% Plots the trajectory history
plot3(histT(:,1),histT(:,2),histT(:,3),...
    '.','MarkerEdgeColor',[1 0 0],'MarkerSize',MS);
end


%%============================Plot Human Arm===============================
% Plots the Human arm reconstruction estimate. Allows comparison between
% desired arm positioning and the actual controlled robot arm.

function PlotHumanArm(shoulder, pointsd)

% Arm points
elbow = pointsd(:,4);
wrist = pointsd(:,5);
hand = pointsd(:,6);
upperArmOri = pointsd(:,7);

% Plotting parameters
LW = 3; MS = 15;

% Plots the estimated user arm for comparison
plot3([shoulder(1),elbow(1),upperArmOri(1)],...
    [shoulder(2),elbow(2),upperArmOri(2)],...
    [shoulder(3),elbow(3),upperArmOri(3)],...
    '--','LineWidth',LW,'Color',[1 0 0],'Marker','.',...
    'MarkerEdgeColor',[1 0 0],'MarkerSize',MS);
plot3([elbow(1),wrist(1),hand(1)],...
    [elbow(2),wrist(2),hand(2)],...
    [elbow(3),wrist(3),hand(3)],...
    '--','LineWidth',LW,'Color',[1 0 0],'Marker','.',...
    'MarkerEdgeColor',[1 0 0],'MarkerSize',MS);
end