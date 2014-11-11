
% Define the system parameters as symbolic variables.
syms m1 m2 m3 m4 m5 m6 l1 l2 l3 th1 th2 th3 g

% Determination of the diagonal mass matrix M for the centers of mass.
massVector = [m1,m2,m3,m4,m5,m6];
numMassCenters = length(massVector);
M = diag(massVector);

% Lpercent is the distance from the manipulator base to each center of mass
% in terms of percent of each linkage length.
%            l1   l2   l3      
Lpercent = [1/2,   0,   0;  % m1
              1,   0,   0;  % m2
              1,   0,   0;  % m3
              1, 1/2,   0;  % m4
              1,   1,   0;  % m5
              1,   1,   1]; % m6 (payload)

% Ltheta is the vector of linkage lengths in the global x direction as a
% function of the relative theta angles.
Ltheta = [l1*cos(th1);
          l2*cos(th1+th2);
          l3*cos(th1+th2+th3)];

% L is the vector of moment arms for each respective center of mass.
L = Lpercent*Ltheta;

% The final symbolic torques caused by the linkage masses and payload.
% Torque caused by each center of mass.
taui = g*M*L;     
% Torque caused by the total arm mass and payload.
tau = g*sum(M*L); 