function HK = HeadKinematics
%% =====================Left Manipulator Kinematics========================
% RoboHAZMAT: Senior Design Project
% Motion Control Team
% Gerardo Bledt
% October 16, 2014
%
%  - Setup of all physical parameters and the kinematic relationship
% between them. Numerically solves for the Homogeneous transformation
% based on the inputed Denavit-Hartenberg convention as determined for the
% robot's Head frame.

%% ============================Numerical Setup=============================
H = struct();
H.Name = 'Head';
H.Field = 'HK';
% Number of Degrees of Freedom
H.DOF = 2;

% Frame points in each frame
H.pts = struct();
% Origin points placeholder
H.pts.o = [];
% Joint points
H.pts.p = [zeros(3,H.DOF);ones(1,H.DOF)];
% Defnining new kinematic points
kP = [];
% Kinematic points, can add points other than joints
H.pts.kP = [H.pts.p,kP];
% Frames for each of the kinematic points first n = DOF are joints
H.pts.frames = [1;2];

% Length of the links
H.d = struct();
% Chest to Head (Neck)
H.d.d0c = 0.371; H.d.dc1 = 0.218;
% Head to eyes
H.d.d12 = 0.0; H.d.d23 = 0.084;

H.pts.o = [0;
    0;
    H.d.d0c + H.d.dc1;
    1];

% Physical system constraints, upper and lower bounds
H.opt = struct();
H.opt.bounds = struct();
H.opt.bounds.lb = [-pi/2,-pi/2.5]; % second may switch with ub
H.opt.bounds.ub = [pi/2,pi/3];

% Weighting on importance of points
H.opt.weightings = zeros(1,size(H.pts.kP,2));

% Theta Angles
H.th = struct();

% Theta Angle Definitions 
H.th.thDef = ['  Neck Yaw  ';'  Neck Pitch'];

% Initial Thetas
H.th.thi = zeros(H.DOF,1);
H.th.thi = [0;0];

%% ========================Mathematical Modeling===========================
% DH Convention
syms th1 th2
H.DH = struct();
H.DH.alphas = [-pi/2;0];
H.DH.thetas = H.th.thi;
H.DH.disps = [0; 0];
H.DH.offsets = [0; H.d.d23];

% Homogeneous transformations
H.DH.H = double(DHTransforms(H.DH));
H.DH.HGo = [ 1, 0, 0,     0;
    0, 1, 0,     0;
    0, 0, 1, H.d.d0c + H.d.dc1;
    0, 0, 0,     1];

%% =====================Create Symbolic Definitions========================
% Creates the symbolic 
H.symbs = struct();
syms th1 th2
H.symbs.thiSym = sym(zeros(H.DOF,1));
H.symbs.alphasSym = sym(zeros(H.DOF,1));
H.symbs.thetasSym = sym([th1; th2]);
for i = 1:H.DOF
    H.symbs.alphasSym(i) = H.DH.alphas(i);
    H.symbs.thiSym(i) = H.th.thi(i);
end

%% ===============Transform each point in the global frame=================
HK = RotateKinematicChain(KinematicSystem(H), zeros(H.DOF, 1));