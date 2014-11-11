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
LM.DOF = 7;

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
%Wrist morots and brackets
LM.d.d67 = 0.0; LM.d.d78 = 0.076;

% Redefining origin points
LM.pts.o = [0,    0,    0;
    0,    0,  LM.d.dc1;
    0, LM.d.d0c, LM.d.d0c;
    1,    1,    1];

% Physical system constraints, upper and lower bounds
LM.b = struct();
LM.b.lb = [-pi,-pi/2,-pi/3,-5*pi/6,-pi,-pi/2,-pi/4];
LM.b.ub = [pi,pi/2,5*pi/6,0,pi,pi/2,pi/4];

% Weighting on importance of points
LM.w = [0;0;0;0;0;0;1];

% Theta Angles
LM.th = struct();
% Shoulder: th1 = Pitch, th2 = Yaw, th3 = Roll
LM.th.th1 = 0; LM.th.th2 = 0; LM.th.th3 = 0;
% Elbow: th4 = Pitch, th5 = Yaw
LM.th.th4 = 0; LM.th.th5 = 0;
% Wrist: th6 = Pitch, th7 = Roll
LM.th.th6 = 0; LM.th.th7 = 0;

% Initial Thetas
LM.th.thi = zeros(LM.DOF,1);
LM.th.thi(1) = pi/2; LM.th.thi(2) = pi/2; LM.th.thi(3) = pi/2;
LM.th.thi(4) = pi/2; LM.th.thi(5) = 0; LM.th.thi(6) = -pi/2;
LM.th.thi(7) = 0;

%% ========================Mathematical Modeling===========================
% DH Convention
LM.DH = struct();
LM.DH.alphas = [pi/2; pi/2; pi/2; pi/2; -pi/2; -pi/2; 0];
LM.DH.thetas = [LM.th.thi(1) + LM.th.th1; LM.th.thi(2) + LM.th.th2;...
    LM.th.thi(3) + LM.th.th3; LM.th.thi(4) + LM.th.th4;...
    LM.th.thi(5) + LM.th.th5; LM.th.thi(6) + LM.th.th6;...
    LM.th.thi(7) + LM.th.th7];
LM.DH.disps = [LM.d.d12; -LM.d.d23; 0; 0; -LM.d.d56; 0; 0];
LM.DH.offsets = [0; 0; -LM.d.d34; 0; 0; LM.d.d67; -LM.d.d78];

% Homogeneous transformations
LM.H.H = double(DHTransforms(LM.DH,false));
LM.H.HGo = [ 0, 1, 0,     0;
    0, 0, 1, LM.d.dc1;
    1, 0, 0, LM.d.d0c;
    0, 0, 0,     1];

%% =============================Simulation ================================
% Transform each point in the global frame
for i = 1:LM.DOF
    % the points in Global Coordinates
    LM.H.HG = LM.H.HGo;
    
    for j = 1:i
        LM.H.HG = LM.H.HG*LM.H.H(:,:,j);
    end
    LM.pts.pG(:,i) = LM.H.HG*LM.pts.p(:,i);
end

LMK = KinematicSystem(LM);