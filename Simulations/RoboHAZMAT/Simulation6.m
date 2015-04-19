function Robot = Simulation6(Robot)
%% ======================IMU Controlled Robot Head=========================
% - An extra feature allows the user to draw trajectories in 3D space using
% gestures. Trajectory tracks the arm's wrist point.

% Sets up the Keyboard Control
[RobotFigure, states] = SetupKeyboardControl(Robot, 2);

% Sets up the Serial communication with the IMUs
[IMUCOM, ~, headControlCOM] = SetupCOM;

% Setup communication with IMU and Head
serialObjIMU = SetupIMUSerial(IMUCOM{2});
% [serialMotorControl, motor] = ...
%     SetupHeadControlSerial(headControlCOM);

% *** This is what must be tuned ***
% Head Control Gains
KC = RotateKinematicChain(Robot.KinematicChains.HK,zeros(3,1));
KC.optimization.weightings = [10;10;10;10];

% Sets up the estimated actual arm position
neck = [0,0,0.589];
% shoulderL = [0,0.179,0.371];
link(1,:) = [.084,0,0]; link(2,:) = [0,0,-0.076];

% History vectors for the actual trajectory histories
trajBuffer = 100;
histT = zeros(trajBuffer,3);

% Waits for the user to be ready to use and initializes the arm
ready = ReadyForUse(RobotFigure);
psi = Reset(serialObjIMU, link, zeros(1,4));

% Constant running while loop
% 1. Gets the simulation state
% 2. Reads the IMU data from the sensor
% 3. Estimates the orientation of the head link
% 4. Reconstructs the user's head and desired points
% 5. Inverse Kinematic optimization to estimate joint angles
% 6. Controls the Robot Head motors
% 7. Rotate and plot the robot, human head, and trajectory
while (ready && states.run)
    
    % Runs the loop through the size of the trajectory history buffer
    for i = 1:trajBuffer
        
        % 1. Gets simulation state
        states = guidata(RobotFigure); if (~states.run), break; end;
        
        % 2. Reads the IMU data from the sensors
        %[q(1,:), reset(1)] = ReadIMUQuaternion(serialObjIMU(1));
        [q(1,:), reset(1)] = ReadWirelessIMU(serialObjIMU, '2');
        
        % 3. Estimates the orientation of the arm links
        [linkRRot, psi] = ...
            EstimateHeadOrientation(link, q, reset, psi);
        
        % 4. Reconstructs the user's arm and desired points
        pointsd = ReconstructHead(neck, linkRRot);
        
        % 5. Inverse Kinematic optimization to estimate joint angles
        X = InverseKinematicOptimization(KC, pointsd);
        
        % 6. Controls the Robot Head motors
%         RobotHeadControl(serialMotorControl, motor, X)
        
        % 7. Rotate and plot the robot, human arm, and trajectory
        KC = RotateKinematicChain(KC, X);
        Robot.KinematicChains.HK = KC;
        RobotPlot(Robot);
        
        PlotHumanHead(neck, pointsd);
        histT = ManageTrajectory(i, histT, KC, RobotFigure, states);
        drawnow;
    end
    
    % If user requests to stop, check again to avoid cutting off Serial
    if (~states.run && ReadyForUse(RobotFigure))
        states.run = 1; guidata(RobotFigure, states);
        psi = Reset(serialObjIMU(1), link, psi);
        KC = RotateKinematicChain(KC, zeros(3,1));
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
        fprintf('Y\n\n   Robot Head Control   \n\n');
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
%q(1,:) = ReadIMUQuaternion(serialObjIMU(1));

        [q(1,:), reset(1)] = ReadWirelessIMU(serialObjIMU, '2');

% Forces a reset
reset = ones(1);

% 2. Estimates the orientation of the arm links
[~, psi] = ...
    EstimateHeadOrientation(link, q, reset, psi);
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