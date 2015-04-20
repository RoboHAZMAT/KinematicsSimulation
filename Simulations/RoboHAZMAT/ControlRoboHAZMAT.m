function Robot = ControlRoboHAZMAT(Robot)
%% =================IMU Controlled Robot Arm Simulation====================
% Simulation that allows the robot to be controlled by IMUs worn on the
% arm. One IMU is worn on the upper arm and one IMU on the forearm. This
% allows for an accurate arm positioning estimation. With the estimate of
% the desired user arm, the robot joint motor angles is calculated through
% inverse kinematics optimization.
%
% - An extra feature allows the user to draw trajectories in 3D space using
% gestures. Trajectory tracks the arm's wrist point.

% Controls for the robot
control = true;     % Allows for control of the motors
rightArm = false;   % Allows control of the right arm
leftArm = false;    % Allows control of the left arm
head = true;        % Allows control of the head

% Sets up the Keyboard Control
[RobotFigure, states] = SetupKeyboardControl(Robot, 2);

%% Setup for the Communication lines on the COM ports
% Sets up the communication with the Dynamixels
if (rightArm || leftArm)
    [dynamixelR, dynamixelL] = DynamixelControlSetup; %#ok<*ASGLU>
end
if (rightArm); DynamixelControl(dynamixelR,[0;0;0;-pi/4;0;0],'r'); end;
if (leftArm); DynamixelControl(dynamixelL,[0;0;0;-pi/4;0;0],'l'); end

% Sets up the Serial communication with the IMUs
[nIMUR, nIMUL, nIMUH] = SetWirelessIMU(rightArm, leftArm, head);
[~, ~, headControlCOM, wirelessIMUCOM, arbotixCOM] = SetupCOM;
if (rightArm || leftArm || head)
    nIMU = [nIMUR, nIMUL, nIMUH];
    serialObjWirelessIMU = SetupWirelessIMUSerial(wirelessIMUCOM, nIMU);
end

% Setup the arbotixCOM port
if (control && (rightArm || leftArm))
    serialObjArbotix = SetupArbotixControlSerial(arbotixCOM);
end

if (control && head)
    % Setup the Head Control
    [serialHeadControl, motor] = SetupHeadControlSerial(headControlCOM);
end

%%
% Specifies the arm and points to be controlled
if (rightArm)
    % Right Arm Control Gains
    KCR = RotateKinematicChain(Robot.KinematicChains.RMK,...
        [-pi/2;zeros(4,1);pi/2]);
    KCR.optimization.weightings = [0;0;0;10;10;10;10;10];
end
if (leftArm)
    % Left Arm Control Gains
    KCL = RotateKinematicChain(Robot.KinematicChains.LMK,...
        [-pi/2;zeros(4,1);pi/2]);
    KCL.optimization.weightings = [0;0;0;10;10;10;10;10];
end

% Sets up the estimated actual arm position
shoulderR = [0,-0.179,0.371]; shoulderL = [0,0.179,0.371];
link(1,:) = [0.279,0,0]; link(2,:) = [0,0,-0.076];
link(3,:) = [0.257,0,0]; link(4,:) = [0,0,-0.076];

% Head Control Gains
if (head)
    KCH = RotateKinematicChain(Robot.KinematicChains.HK,zeros(3,1));
    KCH.optimization.weightings = [10;10;10;10];
    % Sets up the estimated actual head position
    neck = [0,0,0.589];
    linkH(1,:) = [.084,0,0]; linkH(2,:) = [0,0,-0.076];
end

%%
% History vectors for the actual trajectory histories
trajBuffer = 100;
%histTR = zeros(trajBuffer,3);
%histTL = zeros(trajBuffer,3);

% Waits for the user to be ready to use and initializes the arm
ready = ReadyForUse(RobotFigure);
if (rightArm)
    psiR = Reset(serialObjWirelessIMU, link, zeros(1,4), nIMUR);
end
if (leftArm)
    psiL = Reset(serialObjWirelessIMU, link, zeros(1,4), nIMUL);
end
if (head)
    psiH = Reset(serialObjWirelessIMU, linkH, zeros(1,2), nIMUH);
end
% psiH = [0,0];
pause(1);

%% Constant running while loop
% 1. Gets the simulation state
% 2. Reads the IMU data from the sensors
% 3. Estimates the orientation of the arm links
% 4. Reconstructs the user's arm and desired points
% 5. Inverse Kinematic optimization to estimate joint angles
% 6. Rotate and plot the robot, human arm, and trajectory
% 7. Move the physical motors
while (ready && states.run)
    
    % Runs the loop through the size of the trajectory history buffer
    for i = 1:trajBuffer
        %% 1. Gets simulation state
        states = guidata(RobotFigure); if (~states.run), break; end
        
        %% 2. Reads the IMU data from the sensors
        if (rightArm)
            qR = zeros(2,4); resetR = zeros(1, length(nIMUR));
            for j = 1:length(nIMUR)
                [qR(j,:), resetR(j)] = ...
                    ReadWirelessIMU(serialObjWirelessIMU, nIMUR(j));
            end
        end
        if (leftArm)
            qL = zeros(2,4); resetL = zeros(1, length(nIMUL));
            for j = 1:2
                [qL(j,:), resetL(j)] = ...
                    ReadWirelessIMU(serialObjWirelessIMU, nIMUL(j));
            end
        end
        if (head)
            [qH, resetH] = ...
                ReadWirelessIMU(serialObjWirelessIMU, nIMUH);
        end
        
        %% 3. Estimates the orientation of the arm links
        if (rightArm)
            [linkRRot, psiR] = ...
                EstimateArmOrientation(link, qR, resetR, psiR);
        end
        if (leftArm)
            [linkLRot, psiL] = ...
                EstimateArmOrientation(link, qL, resetL, psiL);
        end
        if (head)
            [linkHRot, psiH] = ...
                EstimateHeadOrientation(linkH, qH, resetH, psiH);
        end
        
        %% 4. Reconstructs the user's arm and desired points
        if (rightArm); pointsdR = ReconstructArm(shoulderR, linkRRot); end;
        if (leftArm); pointsdL = ReconstructArm(shoulderL, linkLRot); end;
        if (head); pointsdH = ReconstructHead(neck, linkHRot); end;
        
        %% 5. Inverse Kinematic optimization to estimate joint angles
        if (rightArm); XR = InverseKinematicOptimization(KCR, pointsdR); end;
        if (leftArm); XL = InverseKinematicOptimization(KCL, pointsdL); end;
        if (head); XH = InverseKinematicOptimization(KCH, pointsdH); end;
        
        %% 6. Rotate and plot the robot, human arm, and trajectory
        if (rightArm)
            KCR = RotateKinematicChain(KCR, XR);
            Robot.KinematicChains.RMK = KCR;
        end
        if (leftArm)
            KCL = RotateKinematicChain(KCL, XL);
            Robot.KinematicChains.LMK = KCL;
        end
        if (head)
            KCH = RotateKinematicChain(KCH, XH);
            Robot.KinematicChains.HK = KCH;
        end
        RobotPlot(Robot);
        
        if (rightArm); PlotHumanArm(shoulderR, pointsdR); end;
        if (leftArm); PlotHumanArm(shoulderL, pointsdL); end;
        if (head); PlotHumanHead(neck, pointsdH); end;
        %histTR = ManageTrajectory(i, histTR, KCR, RobotFigure, states);
        %histTL = ManageTrajectory(i, histTL, KCL, RobotFigure, states);
        
        %% 7. Moves the Robot motors
        if (control)
            if (rightArm)
                DynamixelControl(dynamixelR, serialObjArbotix, XR, 'r');
            end
            if (leftArm)
                DynamixelControl(dynamixelL, serialObjArbotix, XL, 'l');
            end
            if (head)
                RobotHeadControl(serialHeadControl, motor, XH);
            end
        end
        drawnow;
    end
    
    % If user requests to stop, check again to avoid cutting off Serial
    if (~states.run && ReadyForUse(RobotFigure))
        states.run = 1; guidata(RobotFigure, states);
        if (rightArm)
            psiR = Reset(serialObjWirelessIMU, link, psiR, nIMUR);
            KCR = RotateKinematicChain(KCR, [-pi/2;zeros(5,1)]);
        end
        if (leftArm)
            psiL = Reset(serialObjWirelessIMU, link, psiL, nIMUL);
            KCL = RotateKinematicChain(KCL, [-pi/2;zeros(5,1)]);
        end
    end;
end

% Close Serial communication
delete(instrfindall);
end


%% ============================Ready For Use===============================
% Makes sure that the user is ready to begin controlling the robotic arm.

function ready = ReadyForUse(RobotFigure)
% Wait for user to be ready or quit if not
states = guidata(RobotFigure);
states.begin = -1;
guidata(RobotFigure, states);
fprintf('Begin control? [Y/N]: ');
SetSimulationControlText(states,'Interactive Simulation','Simulation Paused'...
    ,'','Begin control? [Y/N]:');
while (states.begin ~= 0  && states.begin ~= 1)
    states = guidata(RobotFigure);
    if (states.begin == 1)
        fprintf('Y\n\n   Robot ARM Control   \n\n');
        SetSimulationControlText(states,'Interactive Simulation',...
            'Running Simulation...','Robot Arm Control','{Delete to quit}');
        ready = true;
    elseif (states.begin == 0)
        fprintf('N\n');
        ready = false;
    end
    guidata(RobotFigure, states);
    pause(0.1);
end
end


%% ================================Reset===================================
% Resets the arm orientation and outputs the offset psi angle.

function psi = Reset(serialObjWirelessIMU, link, psi, nIMU)

% Gets the two IMU readings from the sensors
q = zeros(length(nIMU),4);
for j  = 1:length(nIMU)
    q(j,:) = ReadWirelessIMUQuaternion(serialObjWirelessIMU, nIMU(j));
end

% Forces a reset
reset = ones(1,2);

% 2. Estimates the orientation of the arm links
[~, psi] = ...
    EstimateArmOrientation(link, q, reset, psi);
end


%% ==========================Manage Trajectory=============================
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


%% ===========================Plot Human Arm===============================
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

%% ============================Plot Human Arm==============================
% Plots the Human head reconstruction estimate. Allows comparison between
% desired head positioning and the actual controlled robot head.

function PlotHumanHead(neck, pointsd)

% Arm points
face = pointsd(:,3);
faceOri = pointsd(:,4);

% Plotting parameters
LW = 3; MS = 15;

% Plots the estimated user arm for comparison
plot3([neck(1),face(1),faceOri(1)],...
    [neck(2),face(2),faceOri(2)],...
    [neck(3),face(3),faceOri(3)],...
    '--','LineWidth',LW,'Color',[1 0 0],'Marker','.',...
    'MarkerEdgeColor',[1 0 0],'MarkerSize',MS);
end