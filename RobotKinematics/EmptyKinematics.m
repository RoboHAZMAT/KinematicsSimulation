function EK = EmptyKinematics
%% =====================Left Manipulator Kinematics========================
% RoboHAZMAT: Senior Design Project
% Motion Control Team
% Gerardo Bledt
% October 15, 2014
%
% An empty kinematics file used as a placeholder to avoid errors when a
% Kinematic Chain file is required in a function. Has no definitions for
% any of the parameters.

%% ============================Numerical Setup=============================
E = struct();
E.Name = 'Empty';
E.Field = 'EK';
% Number of Degrees of Freedom
E.DOF = 0;

% Frame points in each frame
E.pts = struct();
% Origin points placeholder
E.pts.o = [];
% Joint points
E.pts.p = [];
% Defnining new kinematic points
kP = [];
% Kinematic points, can add points other than joints
E.pts.kP = [E.pts.p,kP];
% Frames for each of the kinematic points first n = DOF are joints
E.pts.frames = [];

% Length of the links
E.d = struct();

% Physical system constraints, upper and lower bounds
E.opt = struct();
E.opt.bounds = struct();
E.opt.bounds.lb = []; % second may switch with ub
E.opt.bounds.ub = [];

% Weighting on importance of points
E.opt.w = [];

% Theta Angles
E.th = struct();

% Theta Angle Definitions 
E.th.thDef = [];

% Initial Thetas
E.th.thi = 0;

%% ========================Mathematical Modeling===========================
% DH Convention
E.DH = struct();
E.DH.alphas = 0;
E.DH.thetas = E.th.thi;
E.DH.disps = 0;
E.DH.offsets = 0;

% Homogeneous transformations
E.DH.H = double(DHTransforms(E.DH));
E.DH.HGo = [ 1, 0, 0, 0;
    0, 1, 0, 0;
    0, 0, 1, 0;
    0, 0, 0, 1];

%% =====================Create Symbolic Definitions========================
% Creates the symbolic 
E.symbs = struct();
E.symbs.thiSym = sym(zeros(E.DOF,1));
E.symbs.alphasSym = sym(zeros(E.DOF,1));
E.symbs.thetasSym = sym([]);
for i = 1:E.DOF
    E.symbs.alphasSym(i) = E.DH.alphas(i);
    E.symbs.thiSym(i) = E.th.thi(i);
end

%% ===============Transform each point in the global frame=================
EK = RotateKinematicChain(KinematicSystem(E), zeros(1, 1));