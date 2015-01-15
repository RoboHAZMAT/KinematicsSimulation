function [H, HT, pointsG, err, error, Gradient, Hessian] = ...
    EquationsOfMotionSym(KC, request)
%% ====================Equations of Motion Symbolic========================
% RoboHAZMAT: Senior Design Project
% Motion Control Team
% Gerardo Bledt
% December 29, 2014
%
% Symbolically solves for the Equations of Motion (EOM) of the Kinematic 
% chain passed in. Finds each of the points in the global frame, calculates
% the error between the actual and desired points, and calculates the 
% symbolic Gradient vector and Hessian matrix.

DH = struct();
DH.thetas = KC.symbolic.thetasSym + KC.symbolic.thiSym;
DH.alphas = KC.symbolic.alphasSym;
DH.disps = KC.DHParams.disps;
DH.offsets = KC.DHParams.offsets;

HT = DHTransformsSym(DH);

% Global transformations for each DOF
H(:,:,1) = KC.DHParams.HGo*HT(:,:,1);
for i = 2:KC.DOF
    H(:,:,i) = H(:,:,i-1)*HT(:,:,i);
end

% Calculation of the points in global frame
points = KC.points.kP;
pointsG = sym(zeros(4,KC.DOF));
for i = 1:size(KC.points.kP,2)
    pointsG(:,i) = H(:,:,KC.points.frames(i))*points(:,i);
end

% Set up symbolic variables
syms w1 w2 w3 w4 w5 w6 w7 w8
syms x1 x2 x3 x4 x5 x6 x7 x8
syms y1 y2 y3 y4 y5 y6 y7 y8
syms z1 z2 z3 z4 z5 z6 z7 z8
syms th1 th2 th3 th4 th5 th6
syms thi1 thi2 thi3 thi4 thi5 thi6

w = [w1;w2;w3;w4;w5;w6;w7;w8];
x = [x1;x2;x3;x4;x5;x6;x7;x8];
y = [y1;y2;y3;y4;y5;y6;y7;y8];
z = [z1;z2;z3;z4;z5;z6;z7;z8];
th = [th1;th2;th3;th4;th5;th6];
% thi = [thi1;thi2;thi3;thi4;thi5;thi6];

% Creates the vector of points that are going to be controlled
% Defaults to all points, but a custom set can be requested as an argument
if (nargin == 2)
    requestedPoints = request;
else
    requestedPoints = zeros(size(KC.points.kP,2),1);
    for i = 1:KC.DOF
        requestedPoints(i,1) = i;
    end
end

% Error between 3D location of desired and actual points
index = 0;
err = sym(zeros(1,3*(length(requestedPoints))));
for i = 1:size(KC.points.kP,2)
    % Adds to the error if it is a point of interest
    if (any(requestedPoints == i))
        err(3*(index) + 1) = w(i)*(x(i) - pointsG(1,i))^2;
        err(3*(index) + 2) = w(i)*(y(i) - pointsG(2,i))^2;
        err(3*(index) + 3) = w(i)*(z(i) - pointsG(3,i))^2;
        index = index + 1;
    end
end

% for i = 1:KC.DOF
%     err(3*(index) + i) = (th(i) - thi(i))^2;
% end

% Sum of the individual errors
error = sym(sum(err));

% Gradient Vector
Gradient = sym(zeros(KC.DOF,1));
for i = 1:KC.DOF
    Gradient(i) = diff(error, th(i));
end

% Hessian Matrix
Hessian = sym(zeros(KC.DOF,KC.DOF));
for i = 1:KC.DOF
    for j = 1:KC.DOF
        Hessian(i,j) = diff(Gradient(i), th(j));
    end
end