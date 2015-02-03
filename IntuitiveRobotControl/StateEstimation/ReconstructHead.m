function pointsd =  ReconstructHead(neck, linkRot)
%% ==========================Reconstruct Head==============================
% RoboHAZMAT: Senior Design Project
% Motion Control Team
% Gerardo Bledt
% January 2, 2015
%
% Reconstructs the user's arm positioning from the rotated link estimates.

% Estimate user's arm positions in 3D 
face = [neck(1,1) + linkRot(1,1),...
    neck(1,2) + linkRot(1,2),neck(1,3) + linkRot(1,3)];
faceOri = [face(1,1) + linkRot(2,1),...
    face(1,2) + linkRot(2,2),face(1,3) + linkRot(2,3)];

% Calculates the desired points for the robot arm
pointsd(:,1) = [neck';1];
pointsd(:,2) = [neck';1];
pointsd(:,3) = [face';1];
pointsd(:,4) = [faceOri';1];