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
MA.pts.o = [];
MA.pts.p = [zeros(3,MA.DOF);ones(1,MA.DOF)];

% Length of the links
MA.d = struct();
% Base height
MA.d.d12 = 0.06; 
% First link
MA.d.d23 = 0.1;
% Second link
MA.d.d34 = 0.08;
% Wrist to gripper tip
MA.d.d45 = 0.08;

% Redefining origin points
MA.pts.o = [0;
    0;
    0;
    1];

% Physical system constraints, upper and lower bounds
MA.b = struct();
MA.b.lb = [-pi/2,-pi/2,-pi/2,-pi/2,-pi/2];
MA.b.ub = [pi/2,pi/2,pi/2,pi/2,pi/2];

% Weighting on importance of points
MA.w = [0;0;0;0;1];

% Theta Angles
MA.th = struct();
% Base: th1 = Yaw, th2 = Pitch 
MA.th.th1 = 0; MA.th.th2 = 0; 
% Elbow: th3 = Pitch
MA.th.th3 = 0;
% Wrist: th4 = Pitch, th5 = Roll
MA.th.th4 = 0; MA.th.th5 = 0;

% Initial Thetas
MA.th.thi = zeros(MA.DOF,1);
MA.th.thi(1) = 0; MA.th.thi(2) = pi/2; MA.th.thi(3) = 0;
MA.th.thi(4) = 0; MA.th.thi(5) = 0;

%% ========================Mathematical Modeling===========================
% DH Convention
MA.DH = struct();
MA.DH.alphas = [pi/2; 0; 0; -pi/2; 0];
MA.DH.thetas = [MA.th.thi(1) + MA.th.th1; MA.th.thi(2) + MA.th.th2;...
    MA.th.thi(3) + MA.th.th3; MA.th.thi(4) + MA.th.th4;...
    MA.th.thi(5) + MA.th.th5];
MA.DH.disps = [MA.d.d12; 0; 0; 0; 0];
MA.DH.offsets = [0; MA.d.d23; MA.d.d34; MA.d.d45; 0];

% Homogeneous transformations
MA.H.H = double(DHTransforms(MA.DH));
MA.H.HGo = [ 1, 0, 0,     0;
    0, 1, 0, 0;
    0, 0, 1, 0;
    0, 0, 0,     1];

%% =============================Simulation ================================
% Transform each point in the global frame
MAK = RotateKinematicChain(KinematicSystem(MA), zeros(MA.DOF,1));