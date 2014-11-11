%% =====================Left Manipulator Kinematics========================
% RoboHAZMAT: Senior Design Project
% Motion Control Team
% Gerardo Bledt
% October 15, 2014
%
%  - Setup of all physical parameters and the kinematic relationship 
% between them. Symbolically solves for the Homogeneous transformation 
% based on the inputed Denavit-Hartenberg convention as determined for the 
% robot's left side manipulator. 
% 
% - A numerical simulation is carried out with measured and estimated
% physical parameters for the actual system.

clear; close all; clc;

%% ===========================Parameter Setup==============================
% Physical Parameters
% Mass of frame
syms m0 m1 m2 m3 m4 m5 m6 m7
% distance from base of frame to center of mass
syms gamma0 gamma1 gamma2 gamma3 gamma4 gamma5 gamma6 gamma7
% distance between frame bases
syms d01 d12 d23 d34 d45 d56 d67 d78
% Rotation angle of the actuated joints
syms th0 th1 th2 th3 th4 th5 th6 th7
% Frame base points
syms p0 p1 p2 p3 p4 p5 p6 p7 p8

% DH Convention 
alphas = [sym(-pi/2);sym(pi/2);sym(pi/2);sym(pi/2);sym(-pi/2);
    sym(pi/2);sym(pi/2);0];
thetas = [th0;sym(th1+pi/2);sym(th2+pi/2);sym(th3+pi/2);th4;th5;
    sym(th6+pi/2);th7];
disps = [d01;d12;d23;d34;d45;d56;d67;d78];
offsets = [0;0;0;0;0;0;0;0];

% Link types
rho = [1;1;1;1;1;1;1];

% Homogeneous transformations
H = DHTransforms(thetas,alphas,disps,offsets);
