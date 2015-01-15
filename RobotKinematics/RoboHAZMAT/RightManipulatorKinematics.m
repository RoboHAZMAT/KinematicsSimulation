function RMK = RightManipulatorKinematics
%% =====================Right Manipulator Kinematics=======================
% RoboHAZMAT: Senior Design Project
% Motion Control Team
% Gerardo Bledt
% October 15, 2014
%
% Setup of all physical parameters and the kinematic relationship between 
% them. Numerically solves for the Homogeneous transformation based on the 
% inputed Denavit-Hartenberg convention as determined for the robot's right 
% side manipulator.

%% ============================Numerical Setup=============================
RM = struct();
RM.Name = 'Right Manipulator';
RM.Field = 'RMK';
% Number of Degrees of Freedom
RM.DOF = 6;

% Frame points in each frame
RM.pts = struct();
% Origin points placeholder
RM.pts.o = [];
% Joint points
RM.pts.p = [zeros(3,RM.DOF);ones(1,RM.DOF)];
% Defnining new kinematic points
kP(:,1) = [-0.076;0;0;1];
kP(:,2) = [-0.076;0;0;1];
% Kinematic points, can add points other than joints
RM.pts.kP = [RM.pts.p,kP];
% Frames for each of the kinematic points first n = DOF are joints
RM.pts.frames = [1;2;3;4;5;6;3;5];

% Length of the links
RM.d = struct();
% Chest to Shoulder
RM.d.d0c = 0.371; RM.d.dc1 = 0.179;
% Shoulder motors and brackets
RM.d.d12 = 0.0; RM.d.d23 = 0.0;
% Upper arm
RM.d.d34 = 0.279;
% Elbow motors and brackets
RM.d.d45 = 0.0;
% Forearm
RM.d.d56 = 0.257;
%Wrist motors and brackets
RM.d.d67 = 0.076;

% Redefining origin points
RM.pts.o = [0,    0,    0;
    0,    0,  -RM.d.dc1;
    0, RM.d.d0c, RM.d.d0c;
    1,    1,    1];

% Physical system constraints, upper and lower bounds
RM.opt = struct();
RM.opt.bounds = struct();
RM.opt.bounds.lb = [-pi, -pi/2, -5*pi/6, -pi, -pi/2, -pi/3];
RM.opt.bounds.ub = [pi/2, pi, pi/2, 0, pi, pi/2];

% Weighting on importance of points
RM.opt.weightings = zeros(1,size(RM.pts.kP,2));

% Theta Angles
RM.th = struct();

% Theta Angle Definitions
RM.th.thDef = ['  Shoulder Pitch';'  Shoulder Yaw  ';'  Shoulder Roll ';...
    '  Elbow Pitch   ';'  Elbow Yaw     ';'  Wrist Pitch   '];

% Initial Thetas
RM.th.thi = zeros(RM.DOF,1);
RM.th.thi = [-pi; -pi/2; pi/2; 0; 0; -pi/2];

%% ========================Mathematical Modeling===========================
% DH Convention
RM.DH = struct();
RM.DH.alphas = [-pi/2; -pi/2; pi/2; -pi/2; pi/2; -pi/2];
RM.DH.thetas = RM.th.thi;
RM.DH.disps = [0; 0; RM.d.d34; 0; RM.d.d56; 0];
RM.DH.offsets = [0; 0; 0; 0; 0; -RM.d.d67];

% Homogeneous transformations
RM.DH.H = double(DHTransforms(RM.DH));
RM.DH.HGo = [ 0, 1, 0,     0;
    0, 0, 1, -RM.d.dc1;
    1, 0, 0, RM.d.d0c;
    0, 0, 0,     1];

%% =====================Create Symbolic Definitions========================
RM.symbs = struct();
syms th1 th2 th3 th4 th5 th6
RM.symbs.thiSym = sym(zeros(RM.DOF,1));
RM.symbs.alphasSym = sym(zeros(RM.DOF,1));
RM.symbs.thetasSym = sym([th1; th2; th3; th4; th5; th6]);
for i = 1:RM.DOF
    RM.symbs.alphasSym(i) = RM.DH.alphas(i);
    RM.symbs.thiSym(i) = RM.th.thi(i);
end

%% ===============Transform each point in the global frame=================
RMK = RotateKinematicChain(KinematicSystem(RM), zeros(RM.DOF, 1));