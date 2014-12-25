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
H.pts.o = [];
H.pts.p = [zeros(3,H.DOF);ones(1,H.DOF)];

% Length of the links
H.d = struct();
% Chest to Head (Neck)
H.d.d0c = 0.371; H.d.dc1 = 0.218;
% Head to eyes
H.d.d12 = 0.0; H.d.d23 = 0.084;
H.pts.p(1,2) = -H.d.d23;

H.pts.o = [   0;
    0;
    H.d.d0c + H.d.dc1;
    1];

% Physical system constraints, upper and lower bounds
H.b = struct();
H.b.lb = [-pi/2,pi/3,0]; % second may switch with ub
H.b.ub = [pi/2,pi/2,0];


% Weighting on importance of points
H.w = [0;1];
% Theta Angles
H.th = struct();
% Body: th0 = Yaw
H.th.th0 = 0;
% Neck: th1 = Yaw, th2 = Pitch
H.th.th1 = 0; H.th.th2 = 0;

% Initial Thetas
H.th.thi = zeros(H.DOF,1);
H.th.thi(1) = -pi/2; H.th.thi(2) = 0;

%% ========================Mathematical Modeling===========================
% DH Convention
H.DH = struct();
H.DH.alphas = [-pi/2; 0];
H.DH.thetas = [H.th.thi(1) + H.th.th1; H.th.thi(2) + H.th.th2];
H.DH.disps = [0; 0];
H.DH.offsets = [0; 0];

% Homogeneous transformations
H.H.H = double(DHTransforms(H.DH));
H.H.HGo = [ 0, 1, 0,     0;
    -1, 0, 0,     0;
    0, 0, 1, H.d.d0c + H.d.dc1;
    0, 0, 0,     1];

% Transform each point in the global frame
HK = RotateKinematicChain(KinematicSystem(H), zeros(H.DOF, 1));