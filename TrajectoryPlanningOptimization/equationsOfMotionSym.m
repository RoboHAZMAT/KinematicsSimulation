function [H, HT, pointsG, error, Gradient, Hessian] = equationsOfMotionSym(KC)
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

syms w4 w5 w6 x4 x5 x6 y4 y5 y6 z4 z5 z6

% Error between 3D location of desired and actual points
err(1) = w4*(x4 - pointsG(1,4))^2;
err(2) = w4*(y4 - pointsG(2,4))^2;
err(3) = w4*(z4 - pointsG(3,4))^2;

err(4) = w5*(x5 - pointsG(1,5))^2;
err(5) = w5*(y5 - pointsG(2,5))^2;
err(6) = w5*(z5 - pointsG(3,5))^2;

err(7) = w6*(x6 - pointsG(1,6))^2;
err(8) = w6*(y6 - pointsG(2,6))^2;
err(9) = w6*(z6 - pointsG(3,6))^2;

error = sum(err);

syms th1 th2 th3 th4 th5 th6
% Gradient Vector
Gradient = sym(zeros(KC.DOF,1));

Gradient(1) = diff(error, th1);
Gradient(2) = diff(error, th2);
Gradient(3) = diff(error, th3);
Gradient(4) = diff(error, th4);
Gradient(5) = diff(error, th5);
Gradient(6) = diff(error, th6);

% Hessian Matrix
Hessian = sym(zeros(KC.DOF,KC.DOF));

Hessian(1,1) = diff(Gradient(1), th1);
Hessian(1,2) = diff(Gradient(1), th2);
Hessian(1,3) = diff(Gradient(1), th3);
Hessian(1,4) = diff(Gradient(1), th4);
Hessian(1,5) = diff(Gradient(1), th5);
Hessian(1,6) = diff(Gradient(1), th6);

Hessian(2,1) = diff(Gradient(2), th1);
Hessian(2,2) = diff(Gradient(2), th2);
Hessian(2,3) = diff(Gradient(2), th3);
Hessian(2,4) = diff(Gradient(2), th4);
Hessian(2,5) = diff(Gradient(2), th5);
Hessian(2,6) = diff(Gradient(2), th6);

Hessian(3,1) = diff(Gradient(3), th1);
Hessian(3,2) = diff(Gradient(3), th2);
Hessian(3,3) = diff(Gradient(3), th3);
Hessian(3,4) = diff(Gradient(3), th4);
Hessian(3,5) = diff(Gradient(3), th5);
Hessian(3,6) = diff(Gradient(3), th6);

Hessian(4,1) = diff(Gradient(4), th1);
Hessian(4,2) = diff(Gradient(4), th2);
Hessian(4,3) = diff(Gradient(4), th3);
Hessian(4,4) = diff(Gradient(4), th4);
Hessian(4,5) = diff(Gradient(4), th5);
Hessian(4,6) = diff(Gradient(4), th6);

Hessian(5,1) = diff(Gradient(5), th1);
Hessian(5,2) = diff(Gradient(5), th2);
Hessian(5,3) = diff(Gradient(5), th3);
Hessian(5,4) = diff(Gradient(5), th4);
Hessian(5,5) = diff(Gradient(5), th5);
Hessian(5,6) = diff(Gradient(5), th6);

Hessian(6,1) = diff(Gradient(6), th1);
Hessian(6,2) = diff(Gradient(6), th2);
Hessian(6,3) = diff(Gradient(6), th3);
Hessian(6,4) = diff(Gradient(6), th4);
Hessian(6,5) = diff(Gradient(6), th5);
Hessian(6,6) = diff(Gradient(6), th6);