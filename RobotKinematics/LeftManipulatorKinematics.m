function LMK = LeftManipulatorKinematics
%% =====================Left Manipulator Kinematics========================
% RoboHAZMAT: Senior Design Project
% Motion Control Team
% Gerardo Bledt
% October 15, 2014
%
%  - Setup of all physical parameters and the kinematic relationship
% between them. Numerically solves for the Homogeneous transformation
% based on the inputed Denavit-Hartenberg convention as determined for the
% robot's left side manipulator.

%% ============================Numerical Setup=============================
LM = struct();
LM.Name = 'Left Manipulator';
LM.Field = 'LMK';
% Number of Degrees of Freedom
LM.DOF = 6;

% Frame points in each frame
LM.pts = struct();
LM.pts.o = [];
LM.pts.p = [zeros(3,LM.DOF);ones(1,LM.DOF)];

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
LM.b = struct();
LM.b.lb = [-pi,-pi/2,-pi/3,-5*pi/6,-pi,-pi/2];
LM.b.ub = [pi,pi/2,5*pi/6,0,pi,pi/2];

% Weighting on importance of points
LM.w = [0;0;0;0;0;1];

% Theta Angles
LM.th = struct();

% Theta Angle Definitions 
LM.th.thDef = ['  Shoulder Pitch';'  Shoulder Yaw  ';'  Shoulder Roll ';...
    '  Elbow Pitch   ';'  Elbow Yaw     ';'  Wrist Pitch   '];

% Initial Thetas
LM.th.thi = zeros(LM.DOF,1);
LM.th.thi(1) = pi/2; LM.th.thi(2) = pi/2; LM.th.thi(3) = pi/2;
LM.th.thi(4) = pi/2; LM.th.thi(5) = 0; LM.th.thi(6) = -pi/2;

% Initial Thetas
LM.th.thiSym = sym(zeros(LM.DOF,1));
LM.th.thiSym(1) = pi/2; LM.th.thiSym(2) = pi/2; LM.th.thiSym(3) = pi/2;
LM.th.thiSym(4) = pi/2; LM.th.thiSym(5) = 0; LM.th.thiSym(6) = -pi/2;

%% ========================Mathematical Modeling===========================
% DH Convention
syms th1 th2 th3 th4 th5 th6
LM.DH = struct();
LM.DH.alphasSym = sym([pi/2; pi/2; pi/2; pi/2; -pi/2; -pi/2]);
LM.DH.thetasSym = sym([th1;th2;th3;th4;th5;th6]);
LM.DH.alphas = [pi/2; pi/2; pi/2; pi/2; -pi/2; -pi/2];
LM.DH.thetas = [LM.th.thi(1); LM.th.thi(2); LM.th.thi(3);...
    LM.th.thi(4); LM.th.thi(5); LM.th.thi(6)];
LM.DH.disps = [LM.d.d12; -LM.d.d23; 0; 0; -LM.d.d56; 0];
LM.DH.offsets = [0; 0; -LM.d.d34; 0; 0; -LM.d.d67];

% Homogeneous transformations
LM.H.H = double(DHTransforms(LM.DH));
LM.H.HGo = [ 0, 1, 0,     0;
    0, 0, 1, LM.d.dc1;
    1, 0, 0, LM.d.d0c;
    0, 0, 0,     1];

%% =============================Simulation ================================
% Transform each point in the global frame
LMK = RotateKinematicChain(KinematicSystem(LM), zeros(LM.DOF, 1));