function MAK = MechatronicArmKinematics
%% ======================Mechatronic Arm Kinematics========================
% RoboHAZMAT: Senior Design Project
% Motion Control Team
% Gerardo Bledt
% December 15, 2014
%
%  - Setup of all physical parameters and the kinematic relationship
% between them. Numerically solves for the Homogeneous transformation
% based on the inputed Denavit-Hartenberg convention as determined for the
% Mechatronic arm prototype.

%% ============================Numerical Setup=============================
MA = struct();
MA.Name = 'Mechatronic Arm';
MA.Field = 'MAK';
% Number of Degrees of Freedom
MA.DOF = 5;

% Frame points in each frame
MA.pts = struct();
% Origin points placeholder
MA.pts.o = [];
% Joint points
MA.pts.p = [zeros(3,MA.DOF);ones(1,MA.DOF)];
% Defnining new kinematic points
kP = [0;0.0271;0.0775;1];
% Kinematic points, can add points other than joints
MA.pts.kP = [MA.pts.p,kP];
% Frames for each of the kinematic points first n = DOF are joints
MA.pts.frames = [1;2;3;4;5;4];

% Length of the links
MA.d = struct();
% Base height
MA.d.d12 = 0.061;
% First link
MA.d.d23 = 0.092;
% Second link
MA.d.d34 = 0.068;
% Wrist to gripper tip
MA.d.d45 = 0.0275;
MA.d.d56 = 0.0775;

% Redefining origin points
MA.pts.o = [0; 0; 0; 1];

% Physical system constraints, upper and lower bounds
MA.opt = struct();
MA.opt.bounds = struct();
MA.opt.bounds.lb = [-pi/2,-pi/2,-pi/2,0,-pi];
MA.opt.bounds.ub = [pi/2,pi/2,pi/2,pi/2,pi];

% Weighting on importance of points
MA.opt.weightings = zeros(1,size(MA.pts.kP,2));

% Theta Angles
MA.th = struct();

% Theta Angle Definitions
MA.th.thDef = ['  Base Yaw   ';'  Base Pitch ';'  Elbow Pitch';...
    '  Wrist Pitch';'  Wrist Roll '];

% Initial Thetas
MA.th.thi = zeros(MA.DOF,1);
MA.th.thi = [0; -pi/2; 0; -pi/2; 0];

%% ========================Mathematical Modeling===========================
% DH Convention
MA.DH = struct();
MA.DH.alphas = [-pi/2; 0; 0; -pi/2; 0];
MA.DH.thetas = [MA.th.thi(1); MA.th.thi(2); MA.th.thi(3); ...
    MA.th.thi(4); MA.th.thi(5)];
MA.DH.disps = [MA.d.d12; 0; 0; 0; MA.d.d56];
MA.DH.offsets = [0; MA.d.d23; MA.d.d34; MA.d.d45; 0];

% Homogeneous transformations
MA.DH.H = double(DHTransforms(MA.DH));
MA.DH.HGo = [ 1, 0, 0, 0;
    0, 1, 0, 0;
    0, 0, 1, 0;
    0, 0, 0, 1];

%% =====================Create Symbolic Definitions========================
MA.symbs = struct();
syms th1 th2 th3 th4 th5
MA.symbs.thiSym = sym(zeros(MA.DOF,1));
MA.symbs.alphasSym = sym(zeros(MA.DOF,1));
MA.symbs.thetasSym = sym([th1; th2; th3; th4; th5]);
for i = 1:MA.DOF
    MA.symbs.alphasSym(i) = MA.DH.alphas(i);
    MA.symbs.thiSym(i) = MA.th.thi(i);
end

%% ===============Transform each point in the global frame=================
MAK = RotateKinematicChain(KinematicSystem(MA), zeros(MA.DOF, 1));