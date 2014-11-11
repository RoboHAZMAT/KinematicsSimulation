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
RM.DOF = 7;

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
RM.d.d67 = 0.0; RM.d.d78 = 0.076;

% Redefining origin points
RM.pts.o = [0,    0,    0;
    0,    0,  -RM.d.dc1;
    0, RM.d.d0c, RM.d.d0c;
    1,    1,    1];

% Physical system constraints, upper and lower bounds
RM.b = struct();
RM.b.lb = [-pi,-pi/2,-5*pi/6,-5*pi/6,-pi,-pi/2,-pi/4];
RM.b.ub = [pi,pi/2,pi/3,0,pi,pi/2,pi/4];

% Weighting on importance of points
RM.w = [0;0;0;0;0;0;1];

% Theta Angles
RM.th = struct();
% Shoulder: th1 = Pitch, th2 = Yaw, th3 = Roll
RM.th.th1 = 0; RM.th.th2 = 0; RM.th.th3 = 0;
% Elbow: th4 = Pitch, th5 = Yaw
RM.th.th4 = 0; RM.th.th5 = 0;
% Wrist: th6 = Pitch, th7 = Roll
RM.th.th6 = 0; RM.th.th7 = 0;

% Initial Thetas
RM.th.thi = zeros(RM.DOF,1);
RM.th.thi(1) = pi/2; RM.th.thi(2) = pi/2; RM.th.thi(3) = pi/2;
RM.th.thi(4) = pi/2; RM.th.thi(5) = 0; RM.th.thi(6) = -pi/2;
RM.th.thi(7) = 0;

% % Masses of the frames
% m0 = 0; m1 = 0; m2 = 0; m3 = 0; m4 = 0; m5 = 0; m6 = 0; m7 = 0;
% masses = [m0;m1;m2;m3;m4;m5;m6;m7];
%
% % Location of centers of mass in frame
% gamma0 = 0; gamma1 = 0; gamma2 = 0; gamma3 = 0;
% gamma4 = 0; gamma5 = 0; gamma6 = 0; gamma7 = 0;
% gammas = [gamma0;gamma1;gamma2;gamma3;gamma4;gamma5;gamma6;gamma7];
%
% % Inertia Matrix
% I0 = zeros(3,3); I1 = zeros(3,3); I2 = zeros(3,3); I3 = zeros(3,3);
% I4 = zeros(3,3); I5 = zeros(3,3); I6 = zeros(3,3); I7 = zeros(3,3);
%
% Link types
% rho = [1;1;1;1;1;1;1];

%% ========================Mathematical Modeling===========================
% DH Convention
RM.DH = struct();
RM.DH.alphas = [pi/2; pi/2; pi/2; pi/2; -pi/2; -pi/2; 0];
RM.DH.thetas = [RM.th.thi(1) + RM.th.th1; RM.th.thi(2) + RM.th.th2;...
    RM.th.thi(3) + RM.th.th3; RM.th.thi(4) + RM.th.th4;...
    RM.th.thi(5) + RM.th.th5; RM.th.thi(6) + RM.th.th6;...
    RM.th.thi(7) + RM.th.th7];
RM.DH.disps = [RM.d.d12; -RM.d.d23; 0; 0; -RM.d.d56; 0; 0];
RM.DH.offsets = [0; 0; -RM.d.d34; 0; 0; RM.d.d67; -RM.d.d78];

% Homogeneous transformations
RM.H.H = double(DHTransforms(RM.DH,false));
RM.H.HGo = [ 0, 1, 0,     0;
    0, 0, 1, -RM.d.dc1;
    1, 0, 0, RM.d.d0c;
    0, 0, 0,     1];

% Transform each point in the global frame
for i = 1:RM.DOF
    % the points in Global Coordinates
    RM.H.HG = RM.H.HGo;
    
    for j = 1:i
        RM.H.HG = RM.H.HG*RM.H.H(:,:,j);
    end
    RM.pts.pG(:,i) = RM.H.HG*RM.pts.p(:,i);
end

RMK = KinematicSystem(RM);