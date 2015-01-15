function LMK = LeftManipulatorKinematics
%% =====================Left Manipulator Kinematics========================
% RoboHAZMAT: Senior Design Project
% Motion Control Team
% Gerardo Bledt
% October 15, 2014
%
% Setup of all physical parameters and the kinematic relationship between 
% them. Numerically solves for the Homogeneous transformation based on the 
% inputed Denavit-Hartenberg convention as determined for the robot's left 
% side manipulator.

%% ============================Numerical Setup=============================
LM = struct();
LM.Name = 'Left Manipulator';
LM.Field = 'LMK';
% Number of Degrees of Freedom
LM.DOF = 6;

% Origin points placeholder
LM.pts.o = [];
% Joint points
LM.pts.p = [zeros(3,LM.DOF);ones(1,LM.DOF)];
% Defnining new kinematic points
kP(:,1) = [-0.076;0;0;1];
kP(:,2) = [-0.076;0;0;1];
% Kinematic points, can add points other than joints
LM.pts.kP = [LM.pts.p,kP];
% Frames for each of the kinematic points first n = DOF are joints
LM.pts.frames = [1;2;3;4;5;6;3;5];

% Length of the links
LM.d = struct();
% Chest to Shoulder
LM.d.d0c = 0.371; LM.d.dc1 = 0.179;
% Shoulder motors and brackets
LM.d.d12 = 0.0; LM.d.d23 = 0.0;
% Upper arm
LM.d.d34 = 0.279;
% Elbow motors and brackets
LM.d.d45 = 0.0;
% Forearm
LM.d.d56 = 0.257;
%Wrist motors and brackets
LM.d.d67 = 0.076;

% Redefining origin points
LM.pts.o = [0,    0,    0;
    0,    0,  LM.d.dc1;
    0, LM.d.d0c, LM.d.d0c;
    1,    1,    1];

% Physical system constraints, upper and lower bounds
LM.opt = struct();
LM.opt.bounds = struct();
LM.opt.bounds.lb = [-pi, -pi/2, -pi/2, -pi, -pi, -pi/3];
LM.opt.bounds.ub = [pi/2, pi, 5*pi/6, 0, pi/2, pi/2];

% Weighting on importance of points
LM.opt.weightings = zeros(1,size(LM.pts.kP,2));

% Theta Angles
LM.th = struct();

% Theta Angle Definitions 
LM.th.thDef = ['  Shoulder Pitch';'  Shoulder Yaw  ';'  Shoulder Roll ';...
    '  Elbow Pitch   ';'  Elbow Yaw     ';'  Wrist Pitch   '];

% Initial Thetas
LM.th.thi = zeros(LM.DOF,1);
LM.th.thi = [-pi; -pi/2; pi/2; 0; 0; -pi/2];

%% ========================Mathematical Modeling===========================
% DH Convention
LM.DH = struct();
LM.DH.alphas = [-pi/2; -pi/2; pi/2; -pi/2; pi/2; -pi/2];
LM.DH.thetas = LM.th.thi;
LM.DH.disps = [0; 0; LM.d.d34; 0; LM.d.d56; 0];
LM.DH.offsets = [0; 0; 0; 0; 0; -LM.d.d67];

% Homogeneous transformations
LM.DH.H = double(DHTransforms(LM.DH));
LM.DH.HGo = [ 0, 1, 0,     0;
    0, 0, 1, LM.d.dc1;
    1, 0, 0, LM.d.d0c;
    0, 0, 0,     1];

%% =====================Create Symbolic Definitions========================
LM.symbs = struct();
syms th1 th2 th3 th4 th5 th6
LM.symbs.thiSym = sym(zeros(LM.DOF,1));
LM.symbs.alphasSym = sym(zeros(LM.DOF,1));
LM.symbs.thetasSym = sym([th1; th2; th3; th4; th5; th6]);
for i = 1:LM.DOF
    LM.symbs.alphasSym(i) = LM.DH.alphas(i);
    LM.symbs.thiSym(i) = LM.th.thi(i);
end

%% ===============Transform each point in the global frame=================
LMK = RotateKinematicChain(KinematicSystem(LM), zeros(LM.DOF, 1));