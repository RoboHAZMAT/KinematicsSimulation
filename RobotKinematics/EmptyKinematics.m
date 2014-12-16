function EK = EmptyKinematics
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
E = struct();
E.Name = 'Empty';
E.Field = 'EK';
% Number of Degrees of Freedom
E.DOF = 0;

% Frame points in each frame 
E.pts = struct();
E.pts.o = [];
E.pts.p = [zeros(3,E.DOF);ones(1,E.DOF)];

% Length of the links
E.d = struct();

% Physical system constraints, upper and lower bounds
E.b = struct();
E.b.lb = [];
E.b.ub = [];

% Weighting on importance of points
E.w = [];

% Theta Angles
E.th = struct();
E.th.thi = [];

% DH Convention 
E.DH = struct();
E.DH.alphas = [0];
E.DH.thetas = [0];
E.DH.disps = [0];
E.DH.offsets = [0];

% Homogeneous transformations
E.H = double(DHTransforms(E.DH));

%% =============================Simulation ================================
% Transform each point in the global frame
for i = 1:E.DOF
    % the points in Global Coordinates
    HG = [];
    for j = 1:i
        HG = HG*E.H(:,:,j);
    end
    E.pts.pG(:,i) = HG*E.pts.p(:,i);
end

EK = KinematicSystem(E);