function pointsd =  ReconstructArm(shoulder, linkRot)
%% ===========================Reconstruct Arm==============================
% RoboHAZMAT: Senior Design Project
% Motion Control Team
% Gerardo Bledt
% January 2, 2015
%
% Reconstructs the user's arm positioning from the rotated link estimates.

% Estimate user's arm positions in 3D 
elbow = [shoulder(1,1) + linkRot(1,1),...
    shoulder(1,2) + linkRot(1,2),shoulder(1,3) + linkRot(1,3)];
wrist = [elbow(1,1) + linkRot(2,1),...
    elbow(1,2) + linkRot(2,2),elbow(1,3) + linkRot(2,3)];
hand = [wrist(1,1) + linkRot(3,1),...
    wrist(1,2) + linkRot(3,2),wrist(1,3) + linkRot(3,3)];

% Calculates the desired points for the robot arm
pointsd(:,3) = [elbow';1];
pointsd(:,4) = [elbow';1];
pointsd(:,5) = [wrist';1];
pointsd(:,6) = [hand';1];

% Plotting parameters
LW = 3; MS = 15;

% Plots the estimated user arm for comparison
plot3([shoulder(1,1),elbow(1,1),wrist(1,1),hand(1,1)],...
    [shoulder(1,2),elbow(1,2),wrist(1,2),hand(1,2)],...
    [shoulder(1,3),elbow(1,3),wrist(1,3),hand(1,3)],...
    '--','LineWidth',LW,'Color',[1 0 0],'Marker','.',...
    'MarkerEdgeColor',[1 0 0],'MarkerSize',MS);
drawnow