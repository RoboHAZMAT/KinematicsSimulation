function [H, HT, pointsG, error, Gradient, Hessian] = ...
    EquationsOfMotionSym(KC, request)
% This program symbolically solves for the Equations of Motion (EOM) of the
% Kinematic chain passed in. Finds each of the points in the global frame,
% calculates the error between the actual and desired points, and
% calculates the symbolib Gradient vector and Hessian matrix.

DH = struct();
DH.thetas = KC.DHParams.thetasSym + KC.thetas.thiSym;
DH.alphas = KC.DHParams.alphasSym;
DH.disps = KC.DHParams.disps;
DH.offsets = KC.DHParams.offsets;

HT = DHTransformsSym(DH);

% Global transformations for each DOF
H(:,:,1) = KC.DHTransforms.HGo*HT(:,:,1);
for i = 2:KC.DOF
    H(:,:,i) = H(:,:,i-1)*HT(:,:,i);
end

% Calculation of the points in global frame
points = [zeros(3,KC.DOF);ones(1,KC.DOF)];
pointsG = sym(zeros(4,KC.DOF));
for i = 1:KC.DOF
    pointsG(:,i) = H(:,:,i)*points(:,i);
end

% Set up symbolic variables
syms w1 w2 w3 w4 w5 w6
syms x1 x2 x3 x4 x5 x6
syms y1 y2 y3 y4 y5 y6
syms z1 z2 z3 z4 z5 z6

w = [w1;w2;w3;w4;w5;w6];
x = [x1;x2;x3;x4;x5;x6];
y = [y1;y2;y3;y4;y5;y6];
z = [z1;z2;z3;z4;z5;z6];

% Creates the vector of points that are going to be controlled
% Defaults to all points, but a custom set can be requested as an argument
if (nargin == 2)
    requestedPoints = request;
else
    requestedPoints = zeros(KC.DOF,1);
    for i = 1:KC.DOF
        requestedPoints(i,1) = i;
    end
end

% Error between 3D location of desired and actual points
index = 0;
err = sym(zeros(3*(length(requestedPoints)),1));
for i = 1:KC.DOF
    % Adds to the error if it is a point of interest
    if (any(requestedPoints == i))
        err(3*(index) + 1) = w(i)*(x(i) - pointsG(1,i))^2;
        err(3*(index) + 2) = w(i)*(y(i) - pointsG(2,i))^2;
        err(3*(index) + 3) = w(i)*(z(i) - pointsG(3,i))^2;
    end
end
% Sum of the individual errors
error = sum(err);

syms th1 th2 th3 th4 th5 th6
th = [th1;th2;th3;th4;th5;th6];

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