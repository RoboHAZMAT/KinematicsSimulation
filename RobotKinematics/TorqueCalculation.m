function tau = TorqueCalculation
% make sure all units are the same 

g = 9.81; % gravity [m/s]

m1 = 0.5097/2.2; % upper arm
m2 = .5/2.2; % motor elbow 1
m3 = .5/2.2; % motor elbow 2
m4 = 0.5097/2.2; % forearm
m5 = .5/2.2; % wrist motors (combined)
m6 = 1/2.2; % payload

l1 = 0.3; % upper arm
l2 = 0.3; % forearm
l3 = 0.1; % wrist to payload

th1 = 0; % shoulder
th2 = 0; % elbow
th3 = 0; % wrist      % keep all th at 0 for stall torque

tau = g*(m6*(l2*cos(th1 + th2) + l1*cos(th1) + l3*cos(th1 + th2 + th3))...
    + m5*(l2*cos(th1 + th2) + l1*cos(th1)) + m4*((l2*cos(th1 + th2))/2 ...
    + l1*cos(th1)) + (l1*m1*cos(th1))/2 + l1*m2*cos(th1) + l1*m3*cos(th1));