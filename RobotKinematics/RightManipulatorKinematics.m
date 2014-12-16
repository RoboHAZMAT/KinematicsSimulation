function RMK = RightManipulatorKinematics
%% =====================Left Manipulator Kinematics========================
% RoboHAZMAT: Senior Design Project
% Motion Control Team
% Gerardo Bledt
% October 15, 2014
%
%  - Setup of all physical parameters and the kinematic relationship
% between them. Numerically solves for the Homogeneous transformation
% based on the inputed Denavit-Hartenberg convention as determined for the
% robot's Right side manipulator.

%% ============================Numerical Setup=============================
RM = struct();
RM.Name = 'Right Manipulator';
RM.Field = 'RMK';
% Number of Degrees of Freedom
RM.DOF = 6;

% Frame points in each frame
RM.pts = struct();
RM.pts.o = [];
RM.pts.p = [zeros(3,RM.DOF);ones(1,RM.DOF)];

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
%Wrist morots and brackets
RM.d.d67 = 0.076;

% Redefining origin points
RM.pts.o = [0,    0,    0;
    0,    0,  -RM.d.dc1;
    0, RM.d.d0c, RM.d.d0c;
    1,    1,    1];

% Physical system constraints, upper and lower bounds
RM.b = struct();
RM.b.lb = [-pi,-pi/2,-5*pi/6,-5*pi/6,-pi,-pi/2];
RM.b.ub = [pi,pi/2,pi/3,0,pi,pi/2];

% Weighting on importance of points
RM.w = [0;0;0;0;0;1];

% Theta Angles
RM.th = struct();
% Shoulder: th1 = Pitch, th2 = Yaw, th3 = Roll
RM.th.th1 = 0; RM.th.th2 = 0; RM.th.th3 = 0;
% Elbow: th4 = Pitch, th5 = Yaw
RM.th.th4 = 0; RM.th.th5 = 0;
% Wrist: th6 = Pitch
RM.th.th6 = 0;

% Initial Thetas
RM.th.thi = zeros(RM.DOF,1);
RM.th.thi(1) = pi/2; RM.th.thi(2) = pi/2; RM.th.thi(3) = pi/2;
RM.th.thi(4) = pi/2; RM.th.thi(5) = 0; RM.th.thi(6) = -pi/2;

%% ========================Mathematical Modeling===========================
% DH Convention
RM.DH = struct();
RM.DH.alphas = [pi/2; pi/2; pi/2; pi/2; -pi/2; -pi/2];
RM.DH.thetas = [RM.th.thi(1) + RM.th.th1; RM.th.thi(2) + RM.th.th2;...
    RM.th.thi(3) + RM.th.th3; RM.th.thi(4) + RM.th.th4;...
    RM.th.thi(5) + RM.th.th5; RM.th.thi(6) + RM.th.th6];
RM.DH.disps = [RM.d.d12; -RM.d.d23; 0; 0; -RM.d.d56; 0];
RM.DH.offsets = [0; 0; -RM.d.d34; 0; 0; -RM.d.d67];

% Homogeneous transformations
RM.H.H = double(DHTransforms(RM.DH));
RM.H.HGo = [ 0, 1, 0,     0;
    0, 0, 1, -RM.d.dc1;
    1, 0, 0, RM.d.d0c;
    0, 0, 0,     1];

% Transform each point in the global frame
RMK = RotateKinematicChain(KinematicSystem(RM), zeros(RM.DOF, 1));